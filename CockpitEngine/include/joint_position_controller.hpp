#ifndef JOINT_POSITION_CONTROLLER_HPP
#define JOINT_POSITION_CONTROLLER_HPP

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <robot.hpp>
#include <filtering.hpp>

namespace jointPositionController {
    static constexpr int FREQ = 1000; // Hz
    static constexpr float I_SAT = 4.0f;
    static constexpr float MAX_SPEED = 0.005f; // m.s^-1
    static constexpr float SCHMIDT_LOW_TO_HIGH = 0.001f; //m
    static constexpr float SCHMIDT_HIGH_TO_LOW = 0.0005f; //m
}

class JointPositionController
{
private:
    Robot* robot_;
    Eigen::VectorXf joint_integral_error_;
    Eigen::VectorXf P_gains_, I_gains_, D_gains_;

    bool schmidt_trigger_state_, is_first_time_;
    std::chrono::steady_clock::time_point t_last_;

public:
    JointPositionController(Robot *robot, const Eigen::VectorXf& P_gains, const Eigen::VectorXf& D_gains, const Eigen::VectorXf& I_gains);

    Eigen::VectorXf computeTorques();
    void lock(){
        if (!is_locked_){
            is_locked_ = true;
            joint_lock_position_ = robot_->kinematics_->q_;
        }
    }
    void unlock(){
        if (is_locked_)
            is_locked_ = false;
    }

    bool is_locked_;
    Eigen::VectorXf joint_lock_position_;
};


#endif // JOINT_POSITION_CONTROLLER_HPP
