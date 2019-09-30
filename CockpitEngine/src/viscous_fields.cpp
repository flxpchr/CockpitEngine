#include "viscous_fields.hpp"

ViscousFields::ViscousFields(Robot* robot, float bmax){
    robot_ = robot;
    b_max_ = bmax;
    old_b_ = 0.0;
    old_t_ = std::chrono::high_resolution_clock::now();
}

ViscousFields::~ViscousFields(){
    delete robot_;
}

float ViscousFields::getB(){
    return old_b_;
}

void ViscousFields::resetB(){
    old_b_ = 0.0;
}

Eigen::Vector3f ViscousFields::computeForces(std::string point_control){
    Eigen::Vector3f out_forces(0,0,0);
    if(robot_->trocar_->is_trocar_found_){
        // Compute viscous coefficient
        float b;
        float v_norm = robot_->kinematics_->lin_speed_filtered_.norm();
        if (v_norm < viscousFields::V_MIN)
            b = b_max_;
        else{
            if(v_norm > viscousFields::V_MAX)
                b = viscousFields::B_MIN;
            else
                b = b_max_ - (b_max_-viscousFields::B_MIN)*((v_norm - viscousFields::V_MIN)/(viscousFields::V_MAX-viscousFields::V_MIN));
        }

        // Filter viscous coefficient
        std::chrono::steady_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        float dt = (t_now - old_t_).count() * 1e-9f;
        float lambda = dt / (dt + viscousFields::TAU);
        b = b*lambda + old_b_ *(1-lambda);
        old_b_ = b;
        old_t_ = t_now;

        // Compute lever ratio
        float alpha;
        if(point_control == "POINT_B")
            alpha = robot_->trocar_->BF_/robot_->trocar_->PF_;
        else{
            if(point_control == "POINT_T")
                alpha = -robot_->trocar_->FT_/robot_->trocar_->PF_;
            else    // POINT_P
                alpha = 1;
        }

        // Saturate lever ratio
        std::min(alpha, -viscousFields::MAX_LEVER_RATIO);
        std::max(alpha, viscousFields::MAX_LEVER_RATIO);

        // Compute lever matrix
        Eigen::Vector3f zI_in_base = robot_->eff_z_axis_;
        Eigen::Matrix3f lever_matrix = alpha*(Eigen::Matrix3f::Identity() - zI_in_base*zI_in_base.transpose())+zI_in_base*zI_in_base.transpose();

        // Compute force
        out_forces = -b * lever_matrix*lever_matrix * robot_->kinematics_->lin_speed_filtered_;
    }
    return out_forces;
}
