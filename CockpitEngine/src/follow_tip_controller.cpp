#include "follow_tip_controller.hpp"

FollowTipController::FollowTipController(Robot* robot_slave, Robot* robot_master, Eigen::Matrix4f calib,
                                 float KP, float KD, float KI, float alpha, float gamma, float v_limit, float s_vel_limit) :
                    p_error_filter_(1,1000), lin_speed_stronger_filter_(5,1000), diff_speed_filter_(2,1000){
    robot_slave_ = robot_slave;
    robot_master_ = robot_master;
    trans_calib_ = calib.block<3,1>(0,3);
    rot_calib_ = calib.block<3,3>(0,0);
    KP_ = KP;
    KD_ = KD;
    KI_ = KI;
    alpha_ = alpha;
    gamma_ = gamma;
    v_limit_ = v_limit;
    s_vel_limit_ = s_vel_limit;

    following_ = false;
    schmidt_trigger_state_ = false;
    integral_error_ = Eigen::Vector3f(0,0,0);
}

FollowTipController::~FollowTipController(){
    delete robot_master_;
    delete robot_slave_;
}

void FollowTipController::updateGoalPosition(){

    // Compute position diff in camera frame
    Eigen::PoseMat pose_in_slave_base(0,0,0,rot_calib_*robot_master_->tip_position_ + trans_calib_);
    Eigen::Vector3f tip_in_cam = robot_slave_->fromBaseToTip(pose_in_slave_base).block<3,1>(0,3);
    Eigen::Vector3f dp = tip_in_cam - tip_in_cam_ini_;

    float FcTt = (rot_calib_ * robot_master_->tip_position_ + trans_calib_ - robot_slave_->trocar_->trocar_position_).norm();

    // Compute v and omega at F
    float vz = followTipController::TAU * dp(2);
    float omegax = -followTipController::TAU * dp(1) / FcTt;
    float omegay = followTipController::TAU * dp(0) / FcTt;

    // Change to point P
    Eigen::Vector3f Vf = Eigen::Vector3f(0,0,vz);
    Eigen::Vector3f omega = Eigen::Vector3f(omegax, omegay,0);
    Eigen::Vector3f PF(0,0,robot_slave_->trocar_->PF_);
    Eigen::Vector3f Vp = Vf + PF.cross(omega);

    // Finally update goal
    std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
    float dt = (t_now - t_last_).count()*1e-9f;
    goal_position_ += dt * robot_slave_->eff_rotation_ * Vp;
}

Eigen::Vector3f FollowTipController::computeForces(){
    Eigen::Vector3f out_force(0,0,0);
    if(following_ && robot_master_->trocar_->is_trocar_found_ && robot_slave_->trocar_->is_trocar_found_){
        if(is_beginning_){
            is_beginning_ = false;
            goal_position_ = robot_slave_->kinematics_->eff_position_;
            Eigen::PoseMat pose_in_slave_base(0,0,0,rot_calib_*robot_master_->tip_position_ + trans_calib_);
            tip_in_cam_ini_ = robot_slave_->fromBaseToTip(pose_in_slave_base).block<3,1>(0,3);
        }else{
            // Update goal
            updateGoalPosition();

            // Compute the errors
            // Calib from master to slave
            Eigen::Vector3f p_error = goal_position_ - robot_slave_->kinematics_->eff_position_;

            Eigen::Vector3f p_error_filtered, lin_speed_more_filtered, diff_speed_filtered;
            p_error_filter_.filter(p_error, p_error_filtered);
            lin_speed_stronger_filter_.filter(robot_slave_->kinematics_->lin_speed_, lin_speed_more_filtered);
            diff_speed_filter_.filter(-robot_slave_->kinematics_->lin_speed_, diff_speed_filtered);

            // Compute dt
            std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
            float dt = (t_now - t_last_).count()*1e-9f;

            // Schmidt Trigger to avoid un/activating all the time the integral term
            if (schmidt_trigger_state_) {
                if( p_error.norm() > followTipController::SCHMIDT_HIGH_TO_LOW ){
                    for(int i=0;i<3;i++)
                        integral_error_(i) += ( 1 + alpha_*pow(std::abs(p_error_filtered(i)), gamma_) * (1/(1+exp(s_vel_limit_*(std::abs(lin_speed_more_filtered(i))-v_limit_)))) ) * p_error(i) * dt;
                }
                else
                    schmidt_trigger_state_ = false;
            }
            else{
                if( p_error.norm() >= followTipController::SCHMIDT_LOW_TO_HIGH){
                    for(int i=0;i<3;i++)
                        integral_error_(i) += ( 1 + alpha_*pow(std::abs(p_error_filtered(i)), gamma_) * (1/(1+exp(s_vel_limit_*(std::abs(lin_speed_more_filtered(i))-v_limit_)))) ) * p_error(i) * dt;
                    schmidt_trigger_state_ = true;
                }
            }

            // Saturate the integral term
            for(unsigned int i=0; i<3; ++i ){
                if(integral_error_(i) >0)
                  integral_error_(i) = std::min(followTipController::I_SAT / KI_, integral_error_(i));
                else
                  integral_error_(i) = std::max(-followTipController::I_SAT / KI_, integral_error_(i));
            }

            // Compute force towards locked pose
            out_force += KP_ * p_error;
            out_force += KD_ * diff_speed_filtered;
            out_force += KI_ * integral_error_;
        }
    }else{
        // Reset filters and integrals
        p_error_filter_.reset(Eigen::Vector3f(0,0,0));
        lin_speed_stronger_filter_.reset(Eigen::Vector3f(0,0,0));
        diff_speed_filter_.reset(Eigen::Vector3f(0,0,0));
        integral_error_ = Eigen::Vector3f(0,0,0);
    }

    // Update time
    t_last_ = std::chrono::high_resolution_clock::now();
    return out_force;
}
