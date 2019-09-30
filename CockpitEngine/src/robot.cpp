#include <robot.hpp>

Robot::Robot(RobotKinematics *kinematics, Instrument *instrument)
    : tip_filter_(5,1000), tip_speed_filter_(15,1000){
    kinematics_ = kinematics;
    instrument_ = instrument;
    trocar_ = new TrocarDetection(kinematics_, instrument_);
    first_time_ = true;
}

Robot::~Robot(){
    delete kinematics_;
    delete instrument_;
}

void Robot::update(){
    old_tip_position_ = tip_position_;

    kinematics_->update();
    trocar_->update();
}

void Robot6d::update(){
    Robot::update();

    eff_z_axis_ = kinematics_->eff_noisy_z_axis_;
    eff_pose_ = kinematics_->eff_noisy_pose_;
    eff_rotation_ = kinematics_->eff_noisy_rotation_;

    tip_position_ = kinematics_->eff_position_ + eff_z_axis_ * instrument_->m_PT;
    tip_filter_.filter(tip_position_, tip_position_filtered_);

    if(first_time_){
        tip_lin_speed_ = Eigen::Vector3f(0,0,0);
        old_time_ = std::chrono::high_resolution_clock::now();
        first_time_ = false;
    }
    else{
        std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        tip_lin_speed_ = (tip_position_ - old_tip_position_)/((t_now-old_time_).count()*1e-9f);
        old_time_ = t_now;
    }
    tip_speed_filter_.filter(tip_lin_speed_, tip_lin_speed_filtered_);
}

void Robot3d::update(){
    Robot::update();

    // Compute with trocar estimation
    if (trocar_->is_trocar_found_){
        eff_z_axis_ = (trocar_->trocar_position_ - kinematics_->eff_position_).normalized();

        Eigen::Vector3f eff_x_axis_old = kinematics_->eff_noisy_pose_.block<3,1>(0,0);
        Eigen::Vector3f eff_y_axis_old = kinematics_->eff_noisy_pose_.block<3,1>(0,1);
        Eigen::Vector3f eff_x_axis = eff_x_axis_old - (eff_x_axis_old.dot(eff_z_axis_))*eff_z_axis_;
        Eigen::Vector3f eff_y_axis = eff_y_axis_old - (eff_y_axis_old.dot(eff_z_axis_))*eff_z_axis_ - (eff_y_axis_old.dot(eff_x_axis))*eff_x_axis;

        eff_rotation_.block<3,1>(0,0) = eff_x_axis;
        eff_rotation_.block<3,1>(0,1) = eff_y_axis;
        eff_rotation_.block<3,1>(0,2) = eff_z_axis_;
        eff_pose_.block<3,3>(0,0) = eff_rotation_;
        eff_pose_.block<3,1>(0,3) = kinematics_->eff_position_;
    // Compute with robot inaccurate & noisy measurement
    }else{
        eff_z_axis_ = kinematics_->eff_noisy_z_axis_;
        eff_pose_ = kinematics_->eff_noisy_pose_;
        eff_rotation_ = kinematics_->eff_noisy_rotation_;
    }
    tip_position_ = kinematics_->eff_position_ + eff_z_axis_ * instrument_->m_PT;
    tip_filter_.filter(tip_position_, tip_position_filtered_);

    if(first_time_){
        tip_lin_speed_ = Eigen::Vector3f(0,0,0);
        old_time_ = std::chrono::high_resolution_clock::now();
        first_time_ = false;
    }
    else{
        std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        tip_lin_speed_ = (tip_position_ - old_tip_position_)/((t_now-old_time_).count()*1e-9f);
        old_time_ = t_now;
    }
    tip_speed_filter_.filter(tip_lin_speed_, tip_lin_speed_filtered_);
}

void Robot::setForce(Eigen::Vector3f force, Eigen::Vector3f momentum){
    kinematics_->qvirtuose_->setForce(force, momentum);
}

void Robot::setTorques(Eigen::VectorXf torques){
    kinematics_->qvirtuose_->setArticularTorques(torques);
}

Eigen::Matrix4f Robot::fromBaseToEff(const Eigen::Matrix4f& pose_in_base){
    return Eigen::PoseMat(pose_in_base).pretransformed(eff_pose_.inverse());
}
Eigen::Matrix4f Robot::fromBaseToTip(const Eigen::Matrix4f& pose_in_base){
    return fromEffToTip(fromBaseToEff(pose_in_base));
}
Eigen::Matrix4f Robot::fromEffToTip(const Eigen::Matrix4f& pose_in_eff){
    return Eigen::PoseMat(pose_in_eff).pretranslated(0,0,-instrument_->m_PT);
}
Eigen::Matrix4f Robot::fromEffToBase(const Eigen::Matrix4f& pose_in_eff){
    return Eigen::PoseMat(pose_in_eff).pretransformed(eff_pose_);
}
Eigen::Matrix4f Robot::fromTipToBase(const Eigen::Matrix4f& pose_in_tip){
    return fromEffToBase(fromTipToEff(pose_in_tip));
}
Eigen::Matrix4f Robot::fromTipToEff(const Eigen::Matrix4f& pose_in_tip){
    return Eigen::PoseMat(pose_in_tip).pretranslated(0,0,instrument_->m_PT);
}
