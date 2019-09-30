#include <robot_kinematics.hpp>

RobotKinematics::RobotKinematics(VirtuoseInterface *qvirtuose)
    : q_filter_(15,1000), qd_filter_(15,1000),
      lin_speed_filter_(15,1000), rot_speed_filter_(15,1000),
        position_filter_(15,1000), eff_z_axis_filter_(15,1000){
    qvirtuose_ = qvirtuose;
    int nb_joints = qvirtuose_->getNbJoints();
    q_.resize(nb_joints);
    qd_.resize(nb_joints);
    q_filtered_.resize(nb_joints);
    qd_filtered_.resize(nb_joints);
    q_.setZero();
    qd_.setZero();
    q_filtered_.setZero();
    qd_filtered_.setZero();
    J_.resize(6,nb_joints);
}

RobotKinematics::~RobotKinematics(){
    delete qvirtuose_;
}

void RobotKinematics::update(){
    // Articular values
    qvirtuose_->getArticularPosition(q_);
    qvirtuose_->getArticularSpeed(qd_);

    // Compute Jacobian
    J_ = qvirtuose_->computeJacobian6D(q_);

    // Cartesian values
    qvirtuose_->getPhysicalPosition(eff_noisy_pose_);
    eff_position_ = eff_noisy_pose_.block<3,1>(0,3);
    eff_noisy_z_axis_ = eff_noisy_pose_.block<3,1>(0,2);
    eff_noisy_rotation_ = eff_noisy_pose_.block<3,3>(0,0);
    qvirtuose_->getPhysicalSpeed(lin_speed_, rot_speed_);

    // Filtered values
    lin_speed_filter_.filter(lin_speed_, lin_speed_filtered_);
    rot_speed_filter_.filter(rot_speed_, rot_speed_filtered_);
    q_filter_.filter(q_, q_filtered_);
    qd_filter_.filter(qd_, qd_filtered_);
    position_filter_.filter(eff_position_, eff_position_filtered_);
    eff_z_axis_filter_.filter(eff_noisy_z_axis_, eff_noisy_z_axis_filtered_);
}
