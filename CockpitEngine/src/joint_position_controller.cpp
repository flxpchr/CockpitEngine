#include <joint_position_controller.hpp>

JointPositionController::JointPositionController(Robot *robot, const Eigen::VectorXf& P_gains,
                        const Eigen::VectorXf& D_gains, const Eigen::VectorXf& I_gains){
    robot_ = robot;
    P_gains_ = P_gains;
    D_gains_ = D_gains;
    I_gains_ = I_gains;

    // Quick tuning values for 6D
//    P_gains << 50,50,70,3,3,3;
//    D_gains << 7,5,3,0.2,0.2,0.1;
//    I_gains << 0,0,0,0,0,0;

    joint_integral_error_.resize(P_gains.size());
    joint_integral_error_.setZero();

    for(int i = 0; i<I_gains_.size(); i++){
        if (I_gains_(i)==0)
            I_gains_(i) = 1e-15f;
    }

    schmidt_trigger_state_ = false;
    is_first_time_ = true;
    is_locked_ = false;
}

Eigen::VectorXf JointPositionController::computeTorques(){

    Eigen::VectorXf out_torques(robot_->kinematics_->q_.size());
    out_torques.setZero();

    std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();

    if(is_locked_){
        // Compute the errors
        Eigen::VectorXf p_error = joint_lock_position_ - robot_->kinematics_->q_;

        // Compute dt
        float dt = (t_now - t_last_).count()*1e-9f;

        // Schmidt Trigger to avoid un/activating all the time the integral term
        if (schmidt_trigger_state_) {
            if( p_error.norm() > jointPositionController::SCHMIDT_HIGH_TO_LOW )
                joint_integral_error_ += p_error * dt;
            else
                schmidt_trigger_state_ = false;
        }
        else{
            if( p_error.norm() >= jointPositionController::SCHMIDT_LOW_TO_HIGH){
                joint_integral_error_ += p_error * dt;
                schmidt_trigger_state_ = true;
            }
        }

        // Saturate the integral term
        for(unsigned int i=0; i<joint_integral_error_.size(); ++i ){
            if(joint_integral_error_(i) >0)
              joint_integral_error_(i) = std::min(jointPositionController::I_SAT / I_gains_(i), joint_integral_error_(i));
            else
              joint_integral_error_(i) = std::max(-jointPositionController::I_SAT / I_gains_(i), joint_integral_error_(i));
        }

        // Compute force towards locked pose
        out_torques += P_gains_.asDiagonal() * p_error;
        out_torques += -1*D_gains_.asDiagonal() * robot_->kinematics_->qd_filtered_;
        out_torques += I_gains_.asDiagonal() * joint_integral_error_;
    }

    // Update time
    t_last_ = t_now;

    return out_torques;
}
