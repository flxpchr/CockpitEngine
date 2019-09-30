#ifndef VISCOUS_FIELDS_HPP
#define VISCOUS_FIELDS_HPP

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <robot.hpp>
#include <chrono>

namespace viscousFields {
    static constexpr float MAX_LEVER_RATIO = 1.6f;
    static constexpr float B_MIN = 0.0f;
    static constexpr float V_MAX = 0.04f; //0.08
    static constexpr float V_MIN = 0.025f; //0.05
    static constexpr float TAU = 0.5f; //s
}
//1-1/(1+std::exp(((x_curr_(i)-viscous_walls_thickness_-cart_min_constraints_(i))*(2/-viscous_walls_thickness_)-1)*6));

class ViscousFields
{
private:
    Robot* robot_;
    float old_b_;
    float b_max_;
    std::chrono::steady_clock::time_point old_t_;

public:
    ViscousFields(Robot *robot, float b_max);
    ~ViscousFields();
    float getB();
    void resetB();
    Eigen::Vector3f computeForces(std::string point_control);
};

#endif // VISCOUS_FIELDS_HPP
