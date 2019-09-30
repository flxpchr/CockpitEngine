#include "gravity_compensation.hpp"

GravityCompensation::GravityCompensation(Robot *robot){
    robot_ = robot;
}

GravityCompensation::~GravityCompensation(){
    delete robot_;
}

Eigen::Vector3f GravityCompensation::computeForces(){
    if(robot_->trocar_->is_trocar_found_){
        // Compute each components of the weight force
        Eigen::Vector3f parallel_weight = robot_->eff_z_axis_.dot(Eigen::Vector3f(0.0f,0.0f,-robot_->instrument_->m_mass*9.81f))*robot_->eff_z_axis_;
        Eigen::Vector3f perpendicular_weight = Eigen::Vector3f(0.0f,0.0f,-robot_->instrument_->m_mass*9.81f) - parallel_weight;

        // Equivalent force created by the moment at the tool frame
        Eigen::Vector3f equivalent_moment_force = robot_->trocar_->GF_/robot_->trocar_->PF_ * perpendicular_weight;

        return -equivalent_moment_force-parallel_weight;
    }
    else{
        // Only compensating the weight
        Eigen::Vector3f compensating_force;
        compensating_force.setZero();
        compensating_force(2) = robot_->instrument_->m_mass*9.81f;
        return compensating_force;
    }
}
