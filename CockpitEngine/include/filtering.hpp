#ifndef FILTERING_HPP
#define FILTERING_HPP

#include <iostream>
#include <algorithm>
#include <vector>
#include <constants.hpp>
#include <Eigen/Dense>

namespace Filters {

class LowPassFilter{
public:
    LowPassFilter(float fc, float f){
        a1_ = (_PI_ *fc - f)/(_PI_ *fc + f);
        b0_ = _PI_ *fc/(_PI_ *fc + f);
        b1_ = b0_;
        first_values_ = true;
    }

    void filter(const Eigen::VectorXf& new_values, Eigen::Ref<Eigen::VectorXf> filtered_values){
        if (first_values_){
            new_values_1_.resize(new_values.size());
            new_values_1_ = new_values;
            filtered_values_.resize(new_values.size());
            filtered_values_ = new_values;
            first_values_ = false;
        }

        filtered_values = b0_*new_values + b1_*new_values_1_ - a1_*filtered_values_;

        new_values_1_ = new_values;
        filtered_values_ = filtered_values;
    }

    void reset(const Eigen::VectorXf& reset_values){
        new_values_1_ = reset_values;
        filtered_values_ = reset_values;
    }
private:
    Eigen::VectorXf filtered_values_, new_values_1_;
    float a1_, b0_, b1_;
    bool first_values_;
};


class Kalman2D{

public:
    Kalman2D(Eigen::Vector2f jerk_std,
             Eigen::Vector2f measur_std,
             Eigen::Matrix<float, 6, 6> init_cov = Eigen::Matrix<float, 6, 6>::Zero()){

        P_k_n_ = init_cov;

        // Measurement Matrix
        H_.fill(0.);
        H_(0,0) = 1.;
        H_(1,1) = 1.;

        // Measurement noise covariance
        R_.fill(0.);
        R_(0,0) = pow(measur_std(0),2);
        R_(1,1) = pow(measur_std(1),2);

        sigma_jerk_ = Eigen::Matrix2f::Zero();
        sigma_jerk_(0,0) = pow(jerk_std(0),2);
        sigma_jerk_(1,1) = pow(jerk_std(1),2);

        is_first_value = true;
    }

    void estimate(Eigen::Vector2f obs, float delta_t, Eigen::Matrix<float, 6, 1> &est){

        // First Step
        if(is_first_value){
            x_k_n_.fill(0.0);
            x_k_n_(0) = obs(0);
            x_k_n_(1) = obs(1);
            is_first_value = false;
        }

        //Previous estimate is k-1th estimate now
        x_k1_ = x_k_n_;
        P_k1_ = P_k_n_;

        predict(delta_t);
        correct(obs);

        est = x_k_n_;
    }

private:
  Eigen::Matrix<float, 6,1> x_k_p_, x_k_n_; // Prediction of the state and following correction by the observation
  Eigen::Matrix<float, 6,1> x_k1_; // State Prediction at the previous time step

  Eigen::Matrix<float, 6, 6> A_, Q_, P_k_p_, P_k_n_, P_k1_;

  Eigen::Matrix2f R_, sigma_jerk_;  // Measurement noise and process noise
  Eigen::Matrix<float, 2, 6> H_;    // Measurement Matrix
  Eigen::Matrix<float, 6, 2> K_;    // Kalman Gain

  float delta_t_;
  bool is_first_value;

    //Modifies the variables associated with change in delta-t
    void delta_change(){
        float dt2_2 = (pow(delta_t_, 2))/2;

        //Recompute A, state transition
        A_ = Eigen::Matrix<float, 6, 6>::Identity();
        A_ <<   1., 0., delta_t_,   0.,   dt2_2,     0.,
                0., 1.,   0.,     delta_t_, 0.,    dt2_2,
                0., 0.,   1.,       0.,   delta_t_,  0.,
                0., 0.,   0.,       1.,     0.,    delta_t_,
                0., 0.,   0.,       0.,     1.,      0.,
                0., 0.,   0.,       0.,     0.,      1.;

        //Recompute Q, covariance process noise
        Eigen::Matrix<float, 6, 2>  G;
        G.fill(0.);

        //Process noise only in the highest order term
        //G(4,0) = delta_t_;
        //G(5,1) = delta_t_;
        //Q_ = G * sigma_jerk_ * G.transpose();

        // Covariance matrix

        float dt5 = (pow(delta_t_, 5))/20;
        float dt4 = (pow(delta_t_, 4))/8;
        float dt3 = (pow(delta_t_, 3))/6;

        Q_.fill(0.);

        Q_ <<   dt5,    0.,     dt4,    0.,     dt3,        0.,
                0.,     dt5,    0.,     dt4,    0.,         dt3,
                dt4,    0.,     dt3*2,  0.,     dt2_2,      0.,
                0.,     dt4,    0.,     dt3*2,  0.,         dt2_2,
                dt3,    0.,     dt2_2,  0.,     delta_t_,   0.,
                0.,     dt3,    0.,     dt2_2,  0.,         delta_t_;

        Q_ = 110000 * Q_;
    }

    void predict(float delta_t){
        delta_t_ = delta_t;
        delta_change();

        //State prediction
        // State vector X = [x y vx vy ax ay]
        x_k_p_ = A_ * x_k1_;

        //Error covariance prediction
        P_k_p_ = A_ * P_k1_ * A_.transpose() + Q_;
    }

    void correct(Eigen::Vector2f z_k){
        //Compute Kalman gain
        K_ = P_k_p_ * H_.transpose() * (H_ * P_k_p_ * H_.transpose() + R_).inverse();

        //Update estimate with observation
        x_k_n_ = x_k_p_ + K_* (z_k - H_ * x_k_p_);

        //Update error cov
        Eigen::Matrix<float, 6, 6> I6 = Eigen::MatrixXf::Identity(6,6);
        P_k_n_ = ( I6 - (K_ * H_)) * P_k_p_;
    }
};



//float Filtering::filterButterworth(float new_value){

//    float filtered_value;
//    std::vector<float> b{0.3375f,0.3375f,0.0f}, a{1.0f,-0.3249f, 0.0f};

//    filtered_value = b[0]*new_values_[0] + b[1]*new_values_[1] + b[2]*new_values_[2] - a[1]*filtered_values_[0] - a[2]*filtered_values_[1];

//    new_values_[2] = new_values_[1];
//    new_values_[1] = new_values_[0];
//    new_values_[0] = new_value;

//    filtered_values_[2] = filtered_values_[1];
//    filtered_values_[1] = filtered_values_[0];
//    filtered_values_[0] = filtered_value;

//    return filtered_value;
//}

}

#endif // FILTERING_HPP
