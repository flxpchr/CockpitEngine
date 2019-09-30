#ifndef CAMERA_ANGLE_CONTROLLER_HPP
#define CAMERA_ANGLE_CONTROLLER_HPP

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <robot.hpp>
#include <chrono>

namespace cameraAngleController {
    static constexpr float I_SAT = 0.2f;
}

class CameraAngleController
{
public:
    CameraAngleController(Robot *robot, float KP, float KD, float KI);
    ~CameraAngleController();
    float computeTorque();

    float ext_goal_th_, ext_goal_th_sat_, ext_q_desired_, ext_delta_q_, ext_delta_q_sat_,ext_angle_to_singularity_, ext_goal_th_no_cone_;

private:
    Robot* robot_;
    std::chrono::steady_clock::time_point old_t_;
    float KP_, KD_, KI_;
    bool is_first_time_;
    float integral_error_;
    float last_desired_;
    float theta_vertical_no_cone_, goal_th_sat_;
};

#endif // CAMERA_ANGLE_CONTROLLER_HPP
