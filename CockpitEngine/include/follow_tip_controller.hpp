#ifndef FOLLOW_TIP_CONTROLLER_HPP
#define FOLLOW_TIP_CONTROLLER_HPP

#include <Eigen/Dense>
#include <iostream>
#include <robot.hpp>

namespace followTipController {
    static constexpr float I_SAT = 4.0f;
    static constexpr float SCHMIDT_LOW_TO_HIGH = 0.001f; //m
    static constexpr float SCHMIDT_HIGH_TO_LOW = 0.0005f; //m
    static constexpr float TAU = 1.5f; //s^-1
}

class FollowTipController
{
public:
    FollowTipController(Robot* robot_slave, Robot* robot_master, Eigen::Matrix4f calib, float KP, float KD, float KI, float alpha, float gamma, float v_limit, float s_vel_limit);
    ~FollowTipController();

    Eigen::Vector3f computeForces();

    void start(){
        if(!following_){
            following_ = true;
            is_beginning_ = true;
        }
    }
    void stop(){
        following_ = false;
        is_beginning_ = true;
    }

private:
    void updateGoalPosition();

    Robot *robot_slave_, *robot_master_;
    Eigen::Matrix3f rot_calib_;
    Eigen::Vector3f trans_calib_;
    float KP_, KD_, KI_;
    float alpha_, gamma_, v_limit_, s_vel_limit_;
    Filters::LowPassFilter diff_speed_filter_, p_error_filter_, lin_speed_stronger_filter_;
    Eigen::Vector3f goal_position_, tip_in_cam_ini_;

    bool following_;
    bool is_beginning_;
    bool schmidt_trigger_state_;
    std::chrono::steady_clock::time_point t_last_;
    Eigen::Vector3f integral_error_;
};

#endif // FOLLOW_TIP_CONTROLLER_HPP
