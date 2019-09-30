#ifndef WALL_CONTROLLER_HPP
#define WALL_CONTROLLER_HPP

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <robot.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Jacobian relating V_M to V_P, M being an arbitrary point on the tool shaft
inline Eigen::Matrix3f jacob_M( float FP, float FM ){
    // FM, FP: algebraic distances between points along the tool z axis
    float ratio = FM / FP;
    return Eigen::Vector3f(ratio,ratio,1).asDiagonal();
}

class WallController
{
private:
    Robot* robot_;

public:
    WallController(Robot *robot);

    Eigen::Vector3f computeForces();
    void saveWallPoint(int id);
    void savePointNormal();
    void resetWall();

    std::vector<Eigen::Vector3f> wall_points;
    Eigen::Vector3f normal_point;
    std::vector<bool> wall_points_set;

    Eigen::Vector3f force_out, tip_force;

private:
    bool wallPointsSet(){
        return wall_points_set[0] && wall_points_set[1] && wall_points_set[2];
    }
};

#endif // WALL_CONTROLLER_HPP
