#define EIGEN_NO_DEBUG

// Http Server (BUG, should be included first)
#include <simpleHttpServer/server_http.hpp>
using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;

// Qt
#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QMenuBar>
// C++ std
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <fstream>

// Robot
#include <virtuose_interface.hpp>
#include <robot_kinematics.hpp>
#include <instrument.hpp>
#include <robot.hpp>
#include <trocar_detection.hpp>


// Controllers
#include <gravity_compensation.hpp>
#include <intelligent_switch.hpp>
#include <viscous_fields.hpp>
#include <camera_angle_controller.hpp>
#include <follow_tip_controller.hpp>
#include <wall_controller.hpp>

// Widgets
#include <mainwindow.hpp>
#include <qvirtuosewidget.h>

// Tools
#include <smartledstripclient.h>
#include <wslogger.h>
#include <filtering.hpp>
#include <keypresseater.hpp>
#include <modes.hpp>

/*
#include "IPC/SharedMemory.h"
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>
*/
//Q_DECLARE_METATYPE(cv::Point2d);
//qRegisterMetaType<const cv::Point2d&>("const cv::Point2d&");


void wait_to_keep_freq(std::chrono::steady_clock::time_point t_start, float freq, bool perfect = false);
void wait_to_keep_freq(std::chrono::steady_clock::time_point t_start, float freq, bool perfect){
    std::chrono::steady_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<int, std::nano> t_ellapsed = t_end - t_start;
    int t_allowed = static_cast<int>(1.0/static_cast<double>(freq)*1e9);
    std::chrono::nanoseconds time_left = std::chrono::nanoseconds(t_allowed) - t_ellapsed;

    if(time_left.count()>0){
        if (perfect){
            std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
            while(t_now - t_end < time_left)
                t_now = std::chrono::high_resolution_clock::now();
        }
        else
            std::this_thread::sleep_for(time_left);
    }
}

void sendRobotStateToLeds(VirtuoseInterface *robot, slsc::SmartLedStripInterface *leds, Robot* virtuose_robot, IntelligentSwitch *intelSwitch, MainWindow* alarm=nullptr){
    slsc::State state = slsc::State::PowerOff;
    if(robot->isReady()){
        int power = 0;
        robot->getPowerOn(&power);
        if(power == 1 && intelSwitch->isLocked()){
            if(intelSwitch->getObstacleState())
                state = slsc::State::LockedBlink;
            else
                state = slsc::State::Locked;
        }
        else if (power == 1 && virtuose_robot->trocar_->isTrocarFound())
            state = slsc::State::TrocarFound;
        else if(power == 1)
            state = slsc::State::PowerOn;
    }
    leds->setState(state);

    if(alarm!=nullptr){
        if(state == slsc::State::LockedBlink){
            alarm->startAlarmFromThread();
        }
        else
            alarm->stopAlarmFromThread();
    }
}


struct INFO
{
    char Name[MAX_PATH];
    int Number;
};

void testMemoryShare(){
    HANDLE FileMappingHandle;
    INFO* FileMapping;
    if ((FileMappingHandle = CreateFileMapping(INVALID_HANDLE_VALUE, 0, PAGE_READWRITE, 0, sizeof(INFO), L"Local\\INFO_MAPPING")) == 0)
    {
        return;
    }

    if ((FileMapping = (INFO*)MapViewOfFile(FileMappingHandle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(INFO))) == 0)
    {
        return;
    }

    strcpy(FileMapping->Name, "DARKVADER");

    FileMapping->Number = 1337;

    printf("FileMapping->Name: %s", FileMapping->Name);
    printf("FileMapping->Number: %d\n", FileMapping->Number);
}

int main(int argc, char *argv[]){

    testMemoryShare();


    /// Qt application and MainWindow
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowState(Qt::WindowMaximized);

    /// WebSocket for oscilloscope on PlotJuggler
    WSLogger* oscillo = new WSLogger(QString("ws://localhost:6666"), false);

    // Add a menu bar
    QMenu *viewMenu = w.menuBar()->addMenu(QObject::tr("&View"));


    /// Create a filter event for pedal actions
    KeyPressEater* kpe = new KeyPressEater();
    bool nPressed = false;

    // Filter the pedals events out
    a.installEventFilter(kpe);

    // Robot connection widget
    QDockWidget *networkDockWidget= new QDockWidget("--- Robot Network ---", &w);
    networkDockWidget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::TopDockWidgetArea);
    QVirtuoseWidget *qvw = new QVirtuoseWidget(nullptr, false);
    qvw->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Minimum);
    networkDockWidget->setWidget(qvw);
    w.addDockWidget(Qt::TopDockWidgetArea, networkDockWidget);
    viewMenu->addAction(networkDockWidget->toggleViewAction());

    // MUTEXES AND THREADS
    std::mutex m_3d, m_3dm, m_view_3d, m_label;
    std::condition_variable cond_var_3d, cond_var_3dm;
    bool thread_condition = true;
    const int ROBOT_COMM_RATE = 1000; // Robots control frequency (Hz)


    // Forces initialization
    Eigen::Vector3f grav_comp_3d_forces(0.0f, 0.0f, 0.0f), grav_comp_3dm_forces(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f lock_3d_forces(0.0f, 0.0f, 0.0f), lock_3dm_forces(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f viscous_3dm_forces(0.0f, 0.0f, 0.0f);

    // PARAMETERS FOR THE 3D CAMERA
    float BP_cam_V3D = 0.200f, PG_cam_V3D = -0.075f, GT_cam_3D = 0.358f; // float BP_cam_V3D = 0.200f, PG_cam_V3D = -0.075f, GT_cam_3D = 0.358f; without light
    float mass_cam_3D = 0.533f;//without light attached (0.533)
    //float mass_cam_3D = 0.933f;//with Hamlyn light attached

    // PARAMETERS FOR THE METAL TOOL
    float BP_tool = 0.163f, PG_tool = -0.042f, GT_tool = 0.362f, mass_tool = 0.17f;

    // Params Intelligent Switch - Medical 3D - tool holder
    float KP_3d = 150.0f, KD_3d = 20.0f, KI_3d = 400.0f;
    float max_error_pos_3d = 0.015f;
    float time_lock_3d = 4.0f;

    // Params Intelligent Switch - Achilles - Camera Holder
    float max_error_pos_3d_cam = 0.015f;
    float time_lock_3d_cam = 2.0f;
    float KP_3d_cam = 150.0f;
    float KD_3d_cam = 20.0f;
    float KI_3d_cam = 400.0f;
    float alpha_cam = 500.0f;
    float gamma_cam = 1.0f;
    float v_limit_cam = 0.01f;
    float s_vel_limit_cam = 500.0f;

    // Params viscosity
    float b_max_3d = 40.0f;

    // Base to base calibration
    Eigen::Matrix4f base_medical3D = Eigen::Matrix4f::Identity();

    base_medical3D << -0.999801758079631f,	0.0198721992686123f,	0.00124106289399217f,	1.12111327781864f,
                      -0.0198406952072909f,	-0.999568747555062f,	0.0216486887569693f,    0.204790570577785f,
                      0.00167073473946748f,	0.0216197735287239f,	0.999764869375793f,     -0.0893999959617944f,
                      0.0f,                	0.0f,                   0.0f,                   1.0f;

    // Starting mode
    Mode current_mode = Mode::unlock;


    // Robot classes and controllers instantiation
    VirtuoseInterface *robot_3d = qvw->getQVirtuose("virtuose_3D_n70"), *robot_3dm = qvw->getQVirtuose("virtuose_3d_n134");

    RobotKinematics *achilles_kin = new RobotKinematics(robot_3d), *medical3d_kin = new RobotKinematics(robot_3dm);

    Instrument *needle_holder = new Instrument(BP_tool, PG_tool, GT_tool, mass_tool), *hd_cam_3D = new Instrument(BP_cam_V3D, PG_cam_V3D, GT_cam_3D, mass_cam_3D);

    Robot3d* achilles_robot = new Robot3d(achilles_kin, hd_cam_3D);
    Robot3d* medical3d_robot = new Robot3d(medical3d_kin, needle_holder);

    GravityCompensation g_comp_3dm(medical3d_robot), g_comp_3d_cam(achilles_robot);

    IntelligentSwitch pose_lock_3d_cam(achilles_robot, max_error_pos_3d_cam, time_lock_3d_cam, KP_3d_cam, KD_3d_cam, KI_3d_cam, alpha_cam, gamma_cam, v_limit_cam, s_vel_limit_cam);
    IntelligentSwitch pose_lock_3dm(medical3d_robot, max_error_pos_3d, time_lock_3d, KP_3d, KD_3d, KI_3d);

    ViscousFields viscous_3dm(medical3d_robot, b_max_3d);

    // Tip Following
    FollowTipController follow_controller_3D(achilles_robot, medical3d_robot, base_medical3D, KP_3d_cam, KD_3d_cam, KI_3d_cam, alpha_cam, gamma_cam, v_limit_cam, s_vel_limit_cam);


    /// Smart LEDs
//    slsc::SmartLedStripHttpClient leds3d(QUrl("http://192.168.100.201"),900); //repeat state every X ms
    slsc::SmartLedStripSerialClient leds3d("COM4",500, &a); //repeat state every X ms
    leds3d.setTimeout(1000); //State stays on for X ms without signals
    leds3d.setBlinkDuration(100);

    slsc:: SmartLedStrip3DMedical leds3dMedical(robot_3dm);
    leds3dMedical.setTimeout(2000);

    /// Qt event connections
    // Jesus Joystick GUI buttons connections
    /*
    QObject::connect(joystick->up_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(0,1,0));} );
    QObject::connect(joystick->down_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(0,-1,0));} );
    QObject::connect(joystick->left_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(1,0,0));} );
    QObject::connect(joystick->right_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(-1,0,0));} );
    QObject::connect(joystick->stop_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(0,0,0));} );
    QObject::connect(joystick->zoom_minus_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(0,0,-1));} );
    QObject::connect(joystick->zoom_plus_button_, &QPushButton::released,[&](){pose_lock_6d.changeDirection(Eigen::Vector3f(0,0,1));} );
    QObject::connect(joystick->record_button_, &QPushButton::released,[&](){pose_lock_6d.record();} );
    QObject::connect(joystick->speed_spin_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double speed){pose_lock_6d.changeTeleopSpeed(speed);} );
    QObject::connect(joystick->KP_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double KP){pose_lock_6d.setKP(KP);} );
    QObject::connect(joystick->KD_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double KD){pose_lock_6d.setKD(KD);} );
    QObject::connect(joystick->KI_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double KI){pose_lock_6d.setKI(KI);} );
    QObject::connect(joystick->alpha_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double alpha){pose_lock_6d.setalpha(alpha);} );
    QObject::connect(joystick->gamma_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double gamma){pose_lock_6d.setgamma(gamma);} );
    QObject::connect(joystick->v_limit_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[&](double v_limit){pose_lock_6d.set_v_limit(v_limit);} );
*/
    // Pedale event connection
    bool new_mode = false;
    QObject::connect(kpe, &KeyPressEater::keyNPressed, [&](bool b) mutable {nPressed = b;});
    QObject::connect(kpe, &KeyPressEater::keyJPressed, [&](bool b) mutable {
        if(b){
            ++current_mode;
            new_mode = true;
        }
    });

    // Exit button connection
    QObject::connect(&w, &MainWindow::closeApplication, [&](){thread_condition = false; a.quit();});


    /// Http server configurations
    HttpServer httpServer;
    httpServer.config.port = 80;

    // Server function for lateral movements
    httpServer.resource["^/move$"]["GET"] = [&oscillo, &m_3d, &pose_lock_3d_cam](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {

        auto map = request->parse_query_string();
        auto dxIt = map.find("dx");
        auto dyIt = map.find("dy");
        std::string content;
        if(dxIt!=map.end()&&dyIt!=map.end()){
            int dx = std::stoi(dxIt->second);
            int dy = std::stoi(dyIt->second);
            std::stringstream ss;
            ss << "dx: "<< dx << "dy: "<< dy;
            std::string msg = ss.str();
            std::cout << "httpserver: " << msg << std::endl;
            content = msg;

            //Interactions
            oscillo->sendMsg("httpserver/delta_x", dx);
            oscillo->sendMsg("httpserver/delta_y", dy);
            oscillo->sendMsg("httpserver/delta_z", 0);
            Eigen::Vector3f v(-dx,-dy,0);

            m_3d.lock();
            pose_lock_3d_cam.changeDirection(v);
            m_3d.unlock();
        }

        *response << "HTTP/1.1 200 OK\r\nContent-Length: " << content.length() << "\r\n\r\n"
                  << content;
    };

    // Server function for zoom in/out motions
    httpServer.resource["^/zoom$"]["GET"] =[&oscillo, &m_3d,&pose_lock_3d_cam](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {
        auto map = request->parse_query_string();
        auto valueIt = map.find("value");
        std::string content;
        if(valueIt!=map.end()){
            int zoom = std::stoi(valueIt->second);
            std::stringstream ss;
            ss << "zoom: "<< zoom;
            content = ss.str();

            Eigen::Vector3f v(0,0,zoom);
            m_3d.lock();
            pose_lock_3d_cam.changeDirection(v);
            m_3d.unlock();
        }
        *response << "HTTP/1.1 200 OK\r\nContent-Length: " << content.length() << "\r\n\r\n"
                  << content;
    };


/// CONTROL THREADS
///
    // Http server thread
    std::thread httpServerThread([&httpServer]() {
        std::cout << "starting server http on port:"<< httpServer.config.port << std::endl;
        // Start server
        httpServer.start();
    });

    std::thread robot_interfacer_3d([&]() {

        int power3d;
        bool just_unlocked = false;
        float time_pressed;
        bool is_first_press = true;
        bool saved_pos_is_set = false;
        std::chrono::steady_clock::time_point t_last;
        bool is_first_obstacle = true;

        while(thread_condition){
            std::chrono::steady_clock::time_point t_start = std::chrono::high_resolution_clock::now();

            if(robot_3d->isReady()){
                std::unique_lock<std::mutex> lock(m_3d);

                /// Update the kinemataic parameters of the robot
                achilles_robot->update();
                robot_3d->getPowerOn(&power3d);

                /// Gravity Compensation Forces
                grav_comp_3d_forces = g_comp_3d_cam.computeForces();

                /// Intelligent Switch
                if(power3d < 1)
                    pose_lock_3d_cam.forceUnlock();

                if(current_mode == Mode::unlock && nPressed && !just_unlocked){
                    pose_lock_3d_cam.forceUnlock();
                    just_unlocked = true;
                }
                if(current_mode == Mode::unlock && !nPressed && just_unlocked){
                    pose_lock_3d_cam.forceLock();
                    just_unlocked = false;
                }

                /// Save a meaningful Position
                if(current_mode == Mode::savepos){

                    if(nPressed){
                        if(is_first_press){
                            is_first_press = false;
                            t_last = std::chrono::high_resolution_clock::now();

                        }
                        std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
                        float dt = (t_now - t_last).count() * 1e-9f;
                        t_last = t_now;
                        time_pressed += dt;
                        if(time_pressed >= 3.0f && time_pressed <= 4.0f){

                        }
                    }else{
                        is_first_press = true;
                    }

                    if(!nPressed && time_pressed >= 3.0f){
                        pose_lock_3d_cam.setSavedPosition();

                        time_pressed = 0.0f;

                    }
                    if(!nPressed && time_pressed < 3.0f && time_pressed > 0.0f){
                        pose_lock_3d_cam.moveToSavedPos();
                        time_pressed = 0.0f;
                        std::cout<<"GOING BACK TO POSITION"<<std::endl;
                    }
                }
                lock_3d_forces = pose_lock_3d_cam.computeForces();

                if( pose_lock_3d_cam.getObstacleState()){
                    is_first_obstacle = false;
                } else {
                    is_first_obstacle = true;
                }


//                oscillo->sendMsg("3d_lock_forces_x", lock_3d_forces(0));
//                oscillo->sendMsg("3d_lock_forces_y", lock_3d_forces(1));
//                oscillo->sendMsg("3d_lock_forces_z", lock_3d_forces(2));
                if(power3d <1 || (current_mode==Mode::follow && nPressed))
                    pose_lock_3d_cam.forceUnlock();

                /// Follow Tip
                Eigen::Vector3f follow_forces(0.0f, 0.0f, 0.0f);
                if(nPressed && current_mode == Mode::follow && robot_3d->isReady()){
                    follow_controller_3D.start();
                    follow_forces = follow_controller_3D.computeForces();
                }else{
                    follow_controller_3D.stop();
//                        if(follow_controller_3D.doEndLock()){
//                            follow_controller_3D.resetEndLock();
//                            pose_lock_3d_cam.forceLock();
//                        }
                }


                if(!achilles_robot->trocar_->is_trocar_found_){
                    grav_comp_3d_forces = grav_comp_3d_forces / 2;
                }

                /// Total Forces
                Eigen::Vector3f total_forces = grav_comp_3d_forces + follow_forces;

                if(pose_lock_3d_cam.isLocked())
                     total_forces += lock_3d_forces;

                /// Total torques
                Eigen::VectorXf total_torques = achilles_kin->J_.transpose() * total_forces;

                /// SEND TORQUES TO THE ROBOT
                achilles_robot->setTorques(total_torques);

                lock.unlock();
                }

            sendRobotStateToLeds(robot_3d,&leds3d,achilles_robot,&pose_lock_3d_cam, &w);
            cond_var_3d.notify_all();
            wait_to_keep_freq(t_start, ROBOT_COMM_RATE, true);
        }
        });

    // 6D robot thread
    // 3D MEDICAL ROBOT - TOOL HOLDER
        std::thread robot_interfacer_3dm([&]() {

            int power3d;
            bool just_unlocked = false;

            while(thread_condition){

                std::chrono::steady_clock::time_point t_start = std::chrono::high_resolution_clock::now();

                if(robot_3dm->isReady()){
                    std::unique_lock<std::mutex> lock(m_3dm);

                    /// Robot interface in
                    medical3d_robot->update();
                    robot_3dm->getPowerOn(&power3d);

                    /// Gravity Compensation
                    grav_comp_3dm_forces = g_comp_3dm.computeForces();

                    /// Intelligent switch
                    if(power3d < 1)
                        pose_lock_3dm.forceUnlock();

                    if(current_mode == Mode::unlock && nPressed && !just_unlocked){
                        pose_lock_3dm.forceUnlock();
                        just_unlocked = true;
                    }
                    if(current_mode == Mode::unlock && !nPressed && just_unlocked){
                        pose_lock_3dm.forceLock();
                        just_unlocked = false;
                    }
                    lock_3dm_forces = pose_lock_3dm.computeForces();

                    /// Viscous fields
                    viscous_3dm_forces = viscous_3dm.computeForces("POINT_P");

                    /// Total forces
                    Eigen::Vector3f total_forces = grav_comp_3dm_forces ;

                    if(pose_lock_3dm.isLocked()){
                        total_forces += lock_3dm_forces;
                        viscous_3dm.resetB();
                    }
                    else
                        total_forces += viscous_3dm_forces;

                    /// Total torques
                    Eigen::VectorXf total_torques = medical3d_kin->J_.transpose() * total_forces;

                    /// Robot interface out
                    medical3d_robot->setTorques(total_torques);

                    lock.unlock();
                }
                sendRobotStateToLeds(robot_3dm,&leds3dMedical,medical3d_robot,&pose_lock_3dm);
                cond_var_3dm.notify_all();
                wait_to_keep_freq(t_start, ROBOT_COMM_RATE, true);
            }
        });

     w.show();

    /// Close everything
    int ret = a.exec();
    thread_condition = false;

    robot_interfacer_3d.join();
    robot_interfacer_3dm.join();

    httpServer.stop();
    httpServerThread.join();


    return ret;
}
