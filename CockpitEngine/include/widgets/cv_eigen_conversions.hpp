#ifndef CV_EIGEN_CONVERSIONS_HPP
#define CV_EIGEN_CONVERSIONS_HPP

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace conversions {

    static void toMatrix3f(const cv::Mat &rvec, Eigen::Matrix3f& rot ){
        cv::Mat rot_cv_mat;
        cv::Rodrigues(rvec, rot_cv_mat);

        for(int i = 0; i < 3; ++i){
            for(int j = 0; j < 3; ++j)
                rot(i, j) = static_cast<float>(rot_cv_mat.at<double>(i, j));
        }
    }

    static void toVector3f(const cv::Mat &tvec, Eigen::Vector3f& vect){
        vect(0) = static_cast<float>(tvec.at<double>(0));
        vect(1) = static_cast<float>(tvec.at<double>(1));
        vect(2) = static_cast<float>(tvec.at<double>(2));
    }

     static void toMatrix4f(const cv::Mat &rvec, const cv::Mat &tvec, Eigen::Matrix4f& mat_out){
        Eigen::Matrix3f rotation;
        Eigen::Vector3f translation;

        toMatrix3f(rvec, rotation);
        toVector3f(tvec, translation);

        mat_out = Eigen::Matrix4f::Identity();
        mat_out.block<3,3>(0,0) = rotation;
        mat_out.block<3,1>(0,3) = translation;
    }
}

#endif // CV_EIGEN_CONVERSIONS_HPP
