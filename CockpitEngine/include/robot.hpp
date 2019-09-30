#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Dense>
#include <robot_kinematics.hpp>
#include <instrument.hpp>
#include <trocar_detection.hpp>
#include <filtering.hpp>
#include <eigen_posemat.hpp>


class Robot{
protected:
    Robot(RobotKinematics* kinematics, Instrument* instrument);
public:
    ~Robot();
    virtual void update();

    /** Commands **/
    void setForce(Eigen::Vector3f force, Eigen::Vector3f momentum);
    void setTorques(Eigen::VectorXf torques);

    /** Base changing functions **/
    Eigen::Matrix4f fromEffToTip(const Eigen::Matrix4f& pose_in_eff);
    Eigen::Matrix4f fromTipToEff(const Eigen::Matrix4f& pose_in_tip);
    Eigen::Matrix4f fromTipToBase(const Eigen::Matrix4f& pose_in_tip);
    Eigen::Matrix4f fromBaseToTip(const Eigen::Matrix4f& pose_in_base);
    Eigen::Matrix4f fromEffToBase(const Eigen::Matrix4f& pose_in_eff);
    Eigen::Matrix4f fromBaseToEff(const Eigen::Matrix4f& pose_in_base);

public:
    RobotKinematics* kinematics_;
    Instrument* instrument_;
    TrocarDetection* trocar_;
    Eigen::Vector3f eff_z_axis_;
    Eigen::Matrix4f eff_pose_;
    Eigen::Matrix3f eff_rotation_;
    Eigen::Vector3f tip_lin_speed_, tip_lin_speed_filtered_;
    Eigen::Vector3f tip_position_, tip_position_filtered_, old_tip_position_;
protected:
    Filters::LowPassFilter tip_filter_, tip_speed_filter_;
    bool first_time_;
    std::chrono::steady_clock::time_point old_time_;
};


class Robot3d : public Robot {
public:
    Robot3d(RobotKinematics* kinematics, Instrument* instrument) : Robot(kinematics, instrument){}
    void update();
};

class Robot6d : public Robot {
public:
    Robot6d(RobotKinematics* kinematics, Instrument* instrument) : Robot(kinematics, instrument){}
    void update();
};

#endif // ROBOT_HPP
