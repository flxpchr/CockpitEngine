#include "intelligent_switch.hpp"
#include <cmath>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

IntelligentSwitch::IntelligentSwitch(Robot* robot, float max_error_pos, float time_to_lock, float KP, float KD, float KI)
        :p_ed_filter_(2,1000), p_error_filter_(1,1000), lin_speed_stronger_filter_(5,1000){
    robot_ = robot;
    integral_error_= Eigen::Vector3f(0,0,0);
    lock_position_ = Eigen::Vector3f(0,0,0);
    prev_lock_position_ = Eigen::Vector3f(0,0,0);
    is_first_time_ = true;
    schmidt_trigger_state_ = false;
    is_locked_ = false;
    force_unlock_ = false;
    force_lock_ = false;
    is_recording_ = false;
    p_error_old_ = Eigen::Vector3f(0,0,0);
    p_error_ = Eigen::Vector3f(0,0,0);
    moving_dir_ = Eigen::Vector3f(0,0,0);
    alpha_ = 0.0f;
    gamma_ = 0.0f;
    v_limit_ = 0.0f;
    s_vel_limit_ = 0.0f;

    saved_position_ = Eigen::Vector3f(0.621f,0.313f,0.126f);

    /// DEBUG
    zaxis_advance_ = 0;
    zaxis_goal_advance_ = 0;
    zaxis_out_force_ = 0;
    zaxis_adv_error_ = 0;
    zaxis_ref_ = Eigen::Vector3f(0,0,0);
    int_comp_ = Eigen::Vector3f(0,0,0);
    moving_dir_ref_ = Eigen::Vector3f(0,0,0);
    compensation_force_ = Eigen::Vector3f(0,0,0);
    proportional_force_ = Eigen::Vector3f(0,0,0);
    damping_force_ = Eigen::Vector3f(0,0,0);

    // Cartesian PID gains
    KP_ = KP;
    KI_ = KI;
    KD_ = KD;
    if (KI_==0)
        KI_ = 1e-15f;

    max_error_pos_ini_ = max_error_pos;
    max_error_pos_ = max_error_pos;
    time_no_move_ = 0.0f;
    time_to_lock_ = time_to_lock;

    is_first_save_ = true;

    moving_z_ = false;
    moving_ = false;
    is_first_move_ = true;
    is_first_move_z_ = true;
    moving_speed_ = 0.02f;

    moving_speed_saved_pos_ = 0.04f;

    movingToSavedPos_ = false;

    obstacle_found_ = false;



    myfile.open ("camera_moves.txt");
}

IntelligentSwitch::IntelligentSwitch(Robot *robot, float max_error_pos, float time_to_lock, float KP, float KD, float KI, float alpha, float gamma, float v_limit, float s_vel_limit):
    IntelligentSwitch(robot,max_error_pos,time_to_lock,KP,KD, KI){

    alpha_ = alpha;
    gamma_ = gamma;
    v_limit_ = v_limit;
    s_vel_limit_ = s_vel_limit;
}


IntelligentSwitch::~IntelligentSwitch(){
    delete robot_;
    myfile.close();
}

bool IntelligentSwitch::isLocked(){
    return is_locked_;
}

Eigen::Vector3f IntelligentSwitch::getLockedPosition(){
    return lock_position_;
}

void IntelligentSwitch::forceUnlock(){
    force_unlock_ = true;
}

void IntelligentSwitch::forceLock(){
    force_lock_ = true;
}

void IntelligentSwitch::changeTeleopSpeed(double speed){
    moving_speed_ = static_cast<float>(speed);
    is_first_move_ = true;
    std::cout << "SPEED " << moving_speed_ << std::endl;
}

void IntelligentSwitch::setKP(double KP){
    KP_ = static_cast<float>(KP);
}

void IntelligentSwitch::setKD(double KD){
    KD_ = static_cast<float>(KD);
}

void IntelligentSwitch::setKI(double KI){
    KI_ = static_cast<float>(KI);
}

void IntelligentSwitch::setalpha(double alpha){
    alpha_ = static_cast<float>(alpha);
}

void IntelligentSwitch::setgamma(double gamma){
    gamma_ = static_cast<float>(gamma);
}

void IntelligentSwitch::set_v_limit(double v_limit){
    v_limit_ = static_cast<float>(v_limit);
}


void IntelligentSwitch::moveToSavedPos(){
    movingToSavedPos_ = true;
    is_first_save_ = true;

}

void IntelligentSwitch::setSavedPosition(){
    saved_position_ = robot_->kinematics_->eff_position_;
}

bool IntelligentSwitch::getObstacleState(){
    return obstacle_found_;
}

void IntelligentSwitch::changeDirection(Eigen::Vector3f dir){
    if(robot_->kinematics_!=nullptr){
        if( dir(0) !=0 || dir(1) !=0 ){
            moving_dir_ = (- dir(0) * ((robot_->eff_rotation_.block<3,1>(0,2)).cross(Eigen::Vector3f (0,0,1))) + dir(1) * (robot_->eff_rotation_.block<3,1>(0,2)).cross(Eigen::Vector3f (1,0,0))).normalized();

            moving_ = true;
            moving_z_ = false;
            is_first_move_z_ = true;
            max_error_pos_ = max_error_pos_ini_;
            moving_speed_ = 0.02f * dir.head(2).norm() * intelligentSwitch::LATERAL_SPEED_RATIO;
            std::cout << "MOVE SIDEWAYS !!!" << std::endl;
        }else{
            if (isnan(dir(2))){
                std::cout << "Got NAN for moving dir_z !!!!" << std::endl;
                return;
            }
            if(dir(2) == 0){
                is_first_move_ = true;
                is_first_move_z_ = true;
                moving_ = false;
//                lock_position_ = robot_->kinematics_->eff_position_;
                max_error_pos_ = max_error_pos_ini_;
                std::cout << "STOP !!!" << std::endl;
            }
            else{
                if(is_first_move_z_){
                    is_first_move_z_ = false;
                    zaxis_ref_ = moving_dir_ref_;
                }
                moving_speed_ = 0.02f;
                moving_ = true;
                moving_z_ = true;
                moving_dir_ = (dir(2) * robot_->eff_rotation_.block<3,1>(0,2)).normalized();
                moving_dir_ref_ = (robot_->eff_rotation_.block<3,1>(0,2)).normalized();
                max_error_pos_ = 0.05f;
                std::cout << "ZOOM !!!" << std::endl;
            }
        }
    }
}



void IntelligentSwitch::record(){
    is_recording_ = !is_recording_;
}

void IntelligentSwitch::update(){
    if(is_first_time_){
        t_last_ = std::chrono::high_resolution_clock::now();
        is_first_time_ = false;
    }


    // Check trocar is found and pose is not locked yet
    if(!is_locked_ && robot_->trocar_->is_trocar_found_){
        if(force_lock_){
            lock_position_ = robot_->kinematics_->eff_position_;
            is_locked_ = true;
            force_lock_ = false;
            time_no_move_ = time_to_lock_;
            std::cout <<"Force locking !" <<std::endl;
        }
        else{
            // Increase counter if xy speed is small
            Eigen::Vector3f robot_speed_in_tool = robot_->eff_rotation_.transpose()*robot_->kinematics_->lin_speed_filtered_;
            if(robot_speed_in_tool.norm()<intelligentSwitch::MAX_SPEED){
                std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
                float dt = (t_now - t_last_).count()*1e-9f;
                time_no_move_ += dt;
            }
            else
                time_no_move_ = 0.0f;

            // If counter is high enough, lock the pose
            if(time_no_move_ >= time_to_lock_){
                lock_position_ = robot_->kinematics_->eff_position_;
                is_locked_ = true;
                std::cout <<"LOCKING !" <<std::endl;
            }
        }
    }
    else
        time_no_move_ = 0.0f;

    // Unlock pose if locked and pulling OR if forced to unlock
    Eigen::Vector3f err_pose_in_tool = robot_->eff_rotation_.transpose()*(lock_position_ - robot_->kinematics_->eff_position_);
    if(is_locked_ &&
            (err_pose_in_tool(2) > max_error_pos_
                || !robot_->trocar_->is_trocar_found_
                || force_unlock_)){
        is_locked_ = false;
        integral_error_ = Eigen::Vector3f(0,0,0);
        time_no_move_ = 0.0f;
        force_unlock_ = false;
        std::cout <<"Un-locking !" <<std::endl;
    }

    // If the lock position is being controlled, move it accordingly
    if(is_locked_ && moving_){
        // Compute dt
        if(is_first_move_){
            last_moving_time_ = std::chrono::high_resolution_clock::now();
            is_first_move_ = false;
            return;
        }
        std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        float dt = (t_now - last_moving_time_).count()*1e-9f;
        last_moving_time_ = t_now;

        if(!moving_z_){
//            std::cout << "moving dir\n" << moving_dir_ << std::endl;
            Eigen::Vector3f circle_dir = ((robot_->trocar_->trocar_position_ - robot_->kinematics_->eff_position_).normalized()).cross( moving_dir_.cross(-robot_->eff_z_axis_)) ; // This definition of circle direction avoids the bug presented at the beginning of the movement with the joystick
            lock_position_ += moving_speed_ * dt * circle_dir;
        }
        else{
            lock_position_ += moving_speed_ * dt * moving_dir_;
        }
    }


    //Move to saved position
    if(is_locked_ && movingToSavedPos_){
        // Compute dt
        if(is_first_save_){
            last_moving_time_ = std::chrono::high_resolution_clock::now();
            is_first_save_ = false;
            return;
        }
        std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        float dt = (t_now - last_moving_time_).count()*1e-9f;
        last_moving_time_ = t_now;


        if((saved_position_-robot_->kinematics_->eff_position_).norm()>intelligentSwitch::TOL_SET_POINT){
            lock_position_ += moving_speed_saved_pos_ * dt * (saved_position_-robot_->kinematics_->eff_position_).normalized();

//            if((lock_position_-robot_->kinematics_->eff_position_).norm()>intelligentSwitch::SAFE_TOL_SAVED_POS
//                    || moving_ || !is_locked_ || obstacle_found_){
//                lock_position_ = robot_->kinematics_->eff_position_;
//                movingToSavedPos_ = false;
//                is_first_save_ = true;
//            }

        }else{
            lock_position_= saved_position_;
            movingToSavedPos_ = false;
            is_first_save_ = true;
        }
    }

    Eigen::Vector3f z = robot_->kinematics_->eff_noisy_z_axis_;
    Eigen::Vector3f error = lock_position_-robot_->kinematics_->eff_position_;

    if(!movingToSavedPos_ && error.norm() > intelligentSwitch::SAFE_TOL && z.dot(error.normalized()) < 0.9){
        //lock_position_ = prev_lock_position_;
        obstacle_found_ = true;

    }else{
        obstacle_found_ = false;
    }

//    std::cout<<"OBSTACLE FOUND\n"<<obstacle_found_<<std::endl;


    prev_lock_position_ = lock_position_;

//    //Safe teleop
//    if((lock_position_-robot_->kinematics_->eff_position_).norm()>intelligentSwitch::SAFE_TOL){
//        moving_ = false;
//        movingToSavedPos_ = false;

//        lock_position_ = robot_->kinematics_->eff_position_;

//        std::cout<<"SAFE TOL REACHED"<<std::endl;

//    }

//






    // record
    if(is_locked_ && is_recording_){
        std::cout << "Recording ..." <<std::endl;
        myfile << robot_->kinematics_->eff_position_(0) << ' ' << robot_->kinematics_->eff_position_(1) << ' ' << robot_->kinematics_->eff_position_(2) << ' '; //current tip coordinates
        myfile << lock_position_(0) << ' ' << lock_position_(1) << ' ' << lock_position_(2) << ' '; // goal tip coordinates
        myfile << KP_ << ' ' << KI_ << ' ' << KD_ << ' '; // gains
        myfile << out_force_(0) << ' ' << out_force_(1) << ' ' << out_force_(2) << ' ';
        myfile << zaxis_advance_ << ' ';
        myfile << zaxis_goal_advance_ << ' ';
        myfile << zaxis_out_force_ << ' ';
        myfile << alpha_ << ' ';
        myfile << int_comp_x_ << ' ' << int_comp_y_ << ' ' << int_comp_z_ << '\n';
    }


}

Eigen::Vector3f IntelligentSwitch::computeForces(){

    update();
    out_force_ = Eigen::Vector3f(0, 0, 0);

    std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();

    if(is_locked_ && (lock_position_-robot_->kinematics_->eff_position_).norm()<intelligentSwitch::SAFE_CRAZY_TOL){
        p_error_ = lock_position_ - robot_->kinematics_->eff_position_;      // Compute the errors
        // Compute time step
        float dt = (t_now - t_last_).count()*1e-9f;

        p_ed_ = (p_error_ - p_error_old_) * (1/dt);             // Error Derivative

        p_error_old_ = p_error_;                                // Store the previous error value
        p_error_filter_.filter(p_error_, p_error_filtered_);    // Position Error filtered
        p_ed_filter_.filter(p_ed_, p_ed_filtered_);             // Filtered Error Derivative
        lin_speed_stronger_filter_.filter(robot_->kinematics_->lin_speed_, lin_speed_more_filtered_); // TODO implement a 1-stage Filter

//        std::cout << "alpha= " << alpha_ << std::endl;
//        std::cout << "KP= " << KP_ << std::endl;
//        std::cout << "KI= " << KI_ << std::endl;
//        std::cout << "KP= " << KD_ << std::endl;

        // Schmidt Trigger to avoid un/activating all the time the integral term
        if (schmidt_trigger_state_) {
            if( p_error_.norm() > intelligentSwitch::SCHMIDT_HIGH_TO_LOW ) {
                /// Cumulative alpha and gamma with vel limit f(v)
                for(int i=0;i<3;i++)
                    integral_error_(i) += ( 1 + alpha_*pow(std::abs(p_error_filtered_(i)), gamma_) * (1/(1+exp(s_vel_limit_*(std::abs(lin_speed_more_filtered_(i))-v_limit_)))) ) * p_error_(i) * dt;
            }
            else
                schmidt_trigger_state_ = false;
        }
        else{
            if( p_error_.norm() >= intelligentSwitch::SCHMIDT_LOW_TO_HIGH){
                /// Accumulative alpha and gamma with vel limit f(v)
                for(int i=0;i<3;i++)
                    integral_error_(i) += ( 1 + alpha_*pow(std::abs(p_error_filtered_(i)), gamma_) * (1/(1+exp(s_vel_limit_*(std::abs(lin_speed_more_filtered_(i))-v_limit_)))) ) * p_error_(i) * dt;

                schmidt_trigger_state_ = true;
            }
        }

        // Saturate the integral term
        for(unsigned int i=0; i<3; ++i ){
            if(integral_error_(i) >0)
              integral_error_(i) = std::min(intelligentSwitch::I_SAT / KI_, integral_error_(i));
            else
              integral_error_(i) = std::max(-intelligentSwitch::I_SAT / KI_, integral_error_(i));
        }

        // Compute force towards locked pose
        out_force_ += KP_ * p_error_;
        out_force_ += KD_ * p_ed_filtered_;
        out_force_ += KI_ * integral_error_;


        /// DEBUG
        proportional_force_= KP_ * p_error_;
        damping_force_= KD_ * p_ed_filtered_;
        compensation_force_= KI_*integral_error_;

        /// Accumulative alpha and gamma with vel limit f(v)
        for(int i=0;i<3;i++)
            int_comp_(i)=alpha_*pow(std::abs(p_error_filtered_(i)),gamma_) * (1/(1+exp(s_vel_limit_*(std::abs(lin_speed_more_filtered_(i))-v_limit_))));

    }else{

        if((lock_position_-robot_->kinematics_->eff_position_).norm()>intelligentSwitch::SAFE_CRAZY_TOL){

            std::cout<<"POSITION ERROR TOO BIG!!!"<<std::endl;
        }


        lock_position_ = robot_->kinematics_->eff_position_;
        p_error_old_ = Eigen::Vector3f(0,0,0);
        integral_error_ = Eigen::Vector3f(0,0,0);

        // Hack to reset filter to zero
        p_ed_filter_.reset(Eigen::Vector3f(0,0,0));
    }

    /// DEBUG
    zaxis_advance_=(robot_->kinematics_->eff_position_).dot(zaxis_ref_);
    zaxis_goal_advance_=(lock_position_).dot(zaxis_ref_);
    zaxis_adv_error_=(zaxis_advance_-zaxis_goal_advance_);
    zaxis_out_force_=out_force_.dot(robot_->eff_rotation_.block<3,1>(0,2));

    // Update time
    t_last_ = t_now;

    return out_force_;
}
