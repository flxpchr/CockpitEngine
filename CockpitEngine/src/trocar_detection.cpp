#include "trocar_detection.hpp"

TrocarDetection::TrocarDetection(RobotKinematics *kinematics, Instrument *instrument):
    A_(&A_buff_[0](0)),
    b_(&b_buff_[0](0))
{
    kinematics_ = kinematics;
    instrument_ = instrument;

    is_trocar_found_ = false;
    nb_step_no_trocar_ = 0;
    trocar_position_ = Eigen::Vector3f(0,0,0);
}

TrocarDetection::~TrocarDetection(){
    delete kinematics_;
    delete instrument_;
}

Eigen::Vector3f TrocarDetection::getTrocarPosition(){
    return trocar_position_;
}

bool TrocarDetection::isTrocarFound(){
    return is_trocar_found_;
}

void TrocarDetection::update(){
    if(isDifferentEnough()){
        // A = rz*rz^T - I3
        Eigen::Matrix3f A = kinematics_->eff_noisy_z_axis_*kinematics_->eff_noisy_z_axis_.transpose() - Eigen::Matrix3f::Identity();
        // b = A*pos
        Eigen::Vector3f b = A*kinematics_->eff_position_;

        // Push back to buffers
        A_buff_.push_back(A);
        b_buff_.push_back(b);
        pose_buff_.push_back(kinematics_->eff_position_);
        z_axis_buff_.push_back(kinematics_->eff_noisy_z_axis_);

        // Wait for buffer to be filled
        if(A_buff_.size()==trocar::BUFF_SIZE)
            computeTrocar();
    }
}

void TrocarDetection::computeTrocar(){
    // Least-square optimisation of the trocar pose
    Eigen::Vector3f estimatedTrocar = (A_.transpose()*A_).inverse()*A_.transpose()*b_;

    // Check if the estimation is acceptable
    if(computeError(estimatedTrocar) < trocar::AVG_ERROR_THRESHOLD
       && (estimatedTrocar-kinematics_->eff_position_).norm() > trocar::MIN_PF_DIST
       && (estimatedTrocar-kinematics_->eff_position_).norm() < instrument_->m_PT
       && ((estimatedTrocar-kinematics_->eff_position_).dot(kinematics_->eff_noisy_z_axis_)) > 0 ){
        is_trocar_found_ = true;
        trocar_position_ = estimatedTrocar;
        nb_step_no_trocar_ = 0;

        PF_ = (trocar_position_ - kinematics_->eff_position_).dot(kinematics_->eff_noisy_z_axis_) ;
        FT_ = instrument_->m_PT - PF_;
        GF_ = PF_ - instrument_->m_PG;
        BF_ = instrument_->m_BP + PF_;
    }
    else{
        nb_step_no_trocar_++;
        if(nb_step_no_trocar_ > trocar::MAX_NB_STEP_NO_TROCAR){
            is_trocar_found_ = false;
            nb_step_no_trocar_ = 0;
            A_buff_.clear();
            b_buff_.clear();
            pose_buff_.clear();
            z_axis_buff_.clear();
        }
    }
}

float TrocarDetection::computeError(const Eigen::Vector3f &estimatedTrocar){
    Eigen::Matrix<float,3*trocar::BUFF_SIZE,1> errors = A_ * estimatedTrocar - b_;

    float avgErr = 0.0;
    for(int i=0; i<3*trocar::BUFF_SIZE; i = i+3)
        avgErr += errors.block(i,0,3,1).norm();
    avgErr /= trocar::BUFF_SIZE;

    return avgErr;
}

bool TrocarDetection::isDifferentEnough(){
    //Should we refresh ? Is the new pos et Z different enough from the previous in buffer
    bool doRefreshTrocar = true;
    for(int i=0; i<pose_buff_.size(); i++){
        float d = (kinematics_->eff_position_ - pose_buff_[i]).norm();
        float alphaD = fabs(kinematics_->eff_noisy_z_axis_.dot(z_axis_buff_[i]));
        if((d < trocar::MIN_LINDIST) && (alphaD > trocar::MAX_ANGDIST)){
            doRefreshTrocar = false;
            break;
        }
    }
    return doRefreshTrocar;
}
