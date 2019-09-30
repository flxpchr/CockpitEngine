#include <wall_controller.hpp>

WallController::WallController(Robot *robot){
    robot_ = robot;

    force_out = Eigen::Vector3f(0,0,0);
    tip_force = Eigen::Vector3f(0,0,0);

    wall_points.resize(3);
    wall_points_set.resize(3);
    resetWall();

    normal_point = Eigen::Vector3f(0,0,0);
}

void WallController::saveWallPoint(int id){
    wall_points[id] = robot_->tip_position_;
    wall_points_set[id] = true;
}

void WallController::savePointNormal(){
    normal_point = robot_->tip_position_;
}

void WallController::resetWall(){
    for(int i=0; i<3; i++)
        wall_points_set[i] = false;
}

Eigen::Vector3f WallController::computeForces(){

    force_out = Eigen::Vector3f(0,0,0);
    tip_force = Eigen::Vector3f(0,0,0);

    if(wallPointsSet()){
        // Wall normal vector
        Eigen::Vector3f n = (wall_points[0] - wall_points[1]).cross(wall_points[2] - wall_points[1]).normalized();

        // Check sign of n with tip position, or gravity...
        Eigen::Vector3f n_bis = normal_point-wall_points[0];
        if(n.dot(n_bis) <0){
            n = -n;
        }

        // Wall rotation matrix
        Eigen::Vector3f v = Eigen::Vector3f::UnitZ().cross(n);
        Eigen::Matrix3f ssc;
        ssc << 0, -v(2), v(1),
               v(2), 0, -v(0),
               -v(1), v(0), 0;
        Eigen::Matrix3f wall_rot = Eigen::Matrix3f::Identity() + ssc + ssc*ssc*(1-n.dot(Eigen::Vector3f::UnitZ()))/(v.norm()*v.norm());

        // Wall pose computation
        Eigen::Matrix4f wall_pose = Eigen::Matrix4f::Identity();
        wall_pose.block<3,1>(0,3) = (wall_points[0]+wall_points[1]+wall_points[2])/3;
        wall_pose.block<3,3>(0,0) = wall_rot;

        // Configuration
        // TODO params this
        float wall_extra = 0.01f;

        // Compute smooth repulsive force
    //    float wall_thickness = 0.01f;
    //    float max_force = 10;
    //    tip_force(2) = max_force*(1-1/(1+exp(((robot_->tip_position_(2)-wall_thickness-z_limit)*(2/-wall_thickness)-1)*6)));

        // Compute spring on z axis
        float K = 600;
        float dIn;
        float distance_to_wall = n.dot(robot_->tip_position_-wall_pose.block<3,1>(0,3))/n.norm();
        if(distance_to_wall- wall_extra>0 )
            dIn = 0;
        else
            dIn = wall_extra-distance_to_wall;
        tip_force = K * dIn * n;

        if(robot_->trocar_->is_trocar_found_){
            // Compute lever matrix
            float alpha = (robot_->instrument_->m_PT-robot_->trocar_->PF_)/robot_->trocar_->PF_;
            Eigen::Vector3f zI_in_base = robot_->eff_z_axis_;
            Eigen::Matrix3f lever_matrix = alpha*(Eigen::Matrix3f::Identity() - zI_in_base*zI_in_base.transpose())+zI_in_base*zI_in_base.transpose();

            // Compute equivalent force at P
            force_out = lever_matrix * tip_force;

            // Instrument jacobians
            Eigen::Matrix3f J_T = jacob_M(-robot_->trocar_->PF_,robot_->trocar_->FT_);
            Eigen::Matrix3f J_B = jacob_M(-robot_->trocar_->PF_,-robot_->trocar_->BF_);

            // Full compensation matrix computation
            Eigen::Matrix3f FCM = -( J_B * Eigen::Vector3f(1,1,-1).asDiagonal() + J_T );

            // Compensation force
            Eigen::Vector3f comp_force = robot_->eff_rotation_ * FCM * robot_->eff_rotation_ * tip_force;
//            force_out += comp_force;
        }
    }

    return force_out;
}
