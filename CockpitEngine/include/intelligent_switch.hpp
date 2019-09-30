#ifndef INTELLIGENT_SWITCH_HPP
#define INTELLIGENT_SWITCH_HPP

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <robot.hpp>
#include <fstream> //mago: to allow .txt file writing
#include <filtering.hpp>

namespace intelligentSwitch {
    static constexpr int FREQ = 1000; // Hz
    static constexpr float I_SAT = 4.0f;
    static constexpr float MAX_SPEED = 0.0075f; // m.s^-1
    static constexpr float SCHMIDT_LOW_TO_HIGH = 0.001f; //m
    static constexpr float SCHMIDT_HIGH_TO_LOW = 0.0005f; //m
    static constexpr float FRICTION_FEEDFWD = 0.0005f; //N
    static constexpr float LATERAL_SPEED_RATIO = 1/300.0f; // pixel^-1
    static constexpr float TOL_SET_POINT = 0.003f; //m
    static constexpr float SAFE_TOL = 0.008f; //m
    static constexpr float SAFE_TOL_SAVED_POS = 0.060f; //m
    static constexpr float SAFE_CRAZY_TOL = 0.150f; //m


}

class IntelligentSwitch
{
private:
    Robot* robot_;
    Eigen::Vector3f integral_error_;
    bool is_locked_, force_unlock_, force_lock_;
    float time_to_lock_, time_no_move_;
    float KP_, KI_, KD_, alpha_, gamma_, v_limit_;
    float KP_zoom_, KI_zoom_, KD_zoom_;
    float max_error_pos_ini_, max_error_pos_;
    float s_vel_limit_;

    bool schmidt_trigger_state_, is_first_time_;
    std::chrono::steady_clock::time_point t_last_;

    bool is_recording_;

    std::chrono::steady_clock::time_point last_moving_time_;
    std::chrono::steady_clock::time_point t_stop_;

    bool moving_, is_first_move_, moving_z_, is_first_move_z_, is_first_save_;
    float moving_speed_, moving_speed_saved_pos_;


    Eigen::Vector3f moving_dir_;
    Eigen::Vector3f moving_dir_ref_;
    Eigen::Vector3f out_force_;
    Eigen::Vector3f zaxis_ref_;
    Eigen::Vector3f lock_position_stop_;
    Eigen::Vector3f current_position_stop_;

    Eigen::Vector3f saved_position_;

    std::ofstream myfile;

    bool movingToSavedPos_, pedal_state_;

    bool obstacle_found_;

    Filters::LowPassFilter p_ed_filter_, p_error_filter_, lin_speed_stronger_filter_;

    void update();

public:
    Eigen::Vector3f lock_position_, prev_lock_position_;
    float zaxis_advance_;
    float zaxis_goal_advance_;
    float zaxis_adv_error_;
    float zaxis_out_force_;
    float int_comp_x_, int_comp_y_, int_comp_z_;
    Eigen::Vector3f p_ed_, p_ed_filtered_, p_error_filtered_, lin_speed_more_filtered_;
    Eigen::Vector3f p_error_old_;
    Eigen::Vector3f p_error_;
    Eigen::Vector3f compensation_force_, damping_force_, proportional_force_, int_comp_;

    IntelligentSwitch(Robot *robot, float max_error_pos, float time_to_lock, float KP, float KD, float KI);
    IntelligentSwitch(Robot *robot, float max_error_pos, float time_to_lock, float KP, float KD, float KI, float alpha, float gamma, float v_limit, float s_vel_limit);
    ~IntelligentSwitch();

    bool isLocked();
    Eigen::Vector3f getLockedPosition();
    Eigen::Vector3f computeForces();
    void forceUnlock();
    void forceLock();

    void changeTeleopSpeed(double speed);
    void changeDirection(Eigen::Vector3f dir);

    void moveToSavedPos();
    void setSavedPosition();


    void record();

    void setKP(double KP);
    void setKI(double KI);
    void setKD(double KD);
    void setalpha(double alpha);
    void setgamma(double gamma);
    void set_v_limit(double v_limit);


    bool getObstacleState();

};

#endif // INTELLIGENT_SWITCH_HPP
