#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include <virtuose_interface.hpp>
#include <Eigen/Dense>
#include <constants.hpp>
#include <filtering.hpp>

class RobotKinematics
{
public:
    RobotKinematics(VirtuoseInterface* qvirtuose);
    ~RobotKinematics();
    void update();

public:
    Eigen::VectorXf q_, q_filtered_;
    Eigen::VectorXf qd_, qd_filtered_;
    Eigen::Vector3f lin_speed_, lin_speed_filtered_;
    Eigen::Vector3f rot_speed_, rot_speed_filtered_;
    Eigen::MatrixXf J_;

    // For the 3D version, this is super noisy
    // Use the one computed thanks to the trocar if you can
    Eigen::Matrix4f eff_noisy_pose_;
    Eigen::Vector3f eff_noisy_z_axis_, eff_noisy_z_axis_filtered_;
    Eigen::Vector3f eff_position_, eff_position_filtered_;
    Eigen::Matrix3f eff_noisy_rotation_;

    VirtuoseInterface* qvirtuose_;

private:
    Filters::LowPassFilter q_filter_, qd_filter_, lin_speed_filter_, rot_speed_filter_, position_filter_, eff_z_axis_filter_;
};

#endif // ROBOT_KINEMATICS_HPP
