#include "camera_angle_controller.hpp"

CameraAngleController::CameraAngleController(Robot* robot, float KP, float KD, float KI){
    KP_ = KP;
    KD_ = KD;
    KI_ = KI;

    if (KI_==0)
        KI_ = 1e-15f;

    robot_ = robot;
    old_t_ = std::chrono::high_resolution_clock::now();
    is_first_time_ = true;
}

CameraAngleController::~CameraAngleController(){
    delete robot_;
}

float CameraAngleController::computeTorque(){
    float out_torque = 0.0f;

    // Compute theoretical goal
    Eigen::Matrix3f inverse_rot_q5 = static_cast<Eigen::Matrix3f>(Eigen::AngleAxisf(-robot_->kinematics_->q_(5), robot_->kinematics_->eff_noisy_z_axis_));
    Eigen::Matrix4f pose_before_q5 = robot_->kinematics_->eff_noisy_pose_;
    pose_before_q5.block<3,3>(0,0) = inverse_rot_q5 * pose_before_q5.block<3,3>(0,0);
    Eigen::Vector3f zp = (Eigen::Matrix3f::Identity() - robot_->kinematics_->eff_noisy_z_axis_*robot_->kinematics_->eff_noisy_z_axis_.transpose())*Eigen::Vector3f::UnitZ();
    zp.normalize();
    float dot = pose_before_q5.block<3,1>(0,1).dot(zp);
    float det = robot_->kinematics_->eff_noisy_z_axis_.dot(pose_before_q5.block<3,1>(0,1).cross(zp));
    float theta_vertical = std::atan2(det,dot);

    // Check proximity to singularity
    float angle_to_singularity = std::abs(std::acos(robot_->kinematics_->eff_noisy_z_axis_.dot(-Eigen::Vector3f::UnitZ())));
    float angle_to_singularity_degrees = angle_to_singularity * 180 / _PI_;
    float max_angle_degrees = 15.0f;
    if (angle_to_singularity_degrees > max_angle_degrees || is_first_time_)
        theta_vertical_no_cone_ = theta_vertical;

    // Remove proximity to limits
    float dmax = 0.1f;
    float qmin = -1.38f;
    float qmax = 1.38f;
    float new_goal_th_sat = std::max(std::min(theta_vertical_no_cone_, qmax - dmax), qmin + dmax);
    if(is_first_time_)
        goal_th_sat_ = new_goal_th_sat;

    // Clever stuff because of -pi == +pi
    if( ((goal_th_sat_ == (qmax - dmax))&&(new_goal_th_sat == (qmin + dmax)))
           || (goal_th_sat_ == (qmin + dmax))&&(new_goal_th_sat == (qmax - dmax)) )
        goal_th_sat_ = goal_th_sat_;
    else
       goal_th_sat_ = new_goal_th_sat;

    // Compute dt
    if(is_first_time_){
        old_t_ = std::chrono::high_resolution_clock::now();
        is_first_time_ = false;
        last_desired_ = goal_th_sat_;
    }
    std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
    float dt = (t_now - old_t_).count()*1e-9f;
    old_t_ = t_now;

    // Filter
    float k = 10;
    float delta_q = k * (goal_th_sat_-last_desired_) * dt;

    // Limit max speed, even more if close to singularity
    float max_delta = 0.0003f, sat_delta_q;
    if (angle_to_singularity_degrees < max_angle_degrees)
        sat_delta_q = std::max(std::min(delta_q, max_delta),-max_delta);
    else
        sat_delta_q = delta_q;

    // Compute q desired
    float q_desired = last_desired_ + sat_delta_q;

    // Final check, just in case
    q_desired = std::max(std::min(q_desired, qmax - dmax), qmin + dmax);

    // For debug
    ext_goal_th_ = theta_vertical;
    ext_goal_th_no_cone_ = theta_vertical_no_cone_;
    ext_goal_th_sat_ = goal_th_sat_;
    ext_q_desired_ = q_desired;
    ext_delta_q_ = delta_q;
    ext_delta_q_sat_ = sat_delta_q;
    ext_angle_to_singularity_ = angle_to_singularity;

    // Compute the errors
    float p_error = q_desired - robot_->kinematics_->q_(5);
    integral_error_ += p_error * dt;

    // Saturate the integral term
    if(KI_ > 1e-6f){
        if(integral_error_ >0)
            integral_error_ = std::min(cameraAngleController::I_SAT / KI_, integral_error_);
        else
            integral_error_ = std::max(-cameraAngleController::I_SAT / KI_, integral_error_);
    }
    else
        integral_error_ = 0.0f;

    // Compute force towards locked pose
    out_torque += KP_ * p_error;
    out_torque += -KD_ * robot_->kinematics_->qd_filtered_(5);
    out_torque += KI_ * integral_error_;

    // Save command
    last_desired_ = q_desired;

    return out_torque;
}
