#ifndef GRAVITY_COMPENSATION_HPP
#define GRAVITY_COMPENSATION_HPP

#include <Eigen/Dense>
#include <iostream>
#include <robot.hpp>

namespace gravityCompensation {
    static constexpr int FREQ = 1000; // Hz
}

class GravityCompensation
{
public:
    GravityCompensation(Robot* robot);
    ~GravityCompensation();

    float distanceToTrocar();
    Eigen::Vector3f computeForces();

private:
    Robot* robot_;
};

#endif // GRAVITY_COMPENSATION_HPP
