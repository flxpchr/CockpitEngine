#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

#include <widgets/cv_eigen_conversions.hpp>

namespace Eigen {
typedef Matrix<float, 6, 1>    Vector6f;
}

class Registration
{
public:
  Registration(std::string calibration_filename);
  ~Registration();

  void update3dPosition(cv::Point3f position);
  void update2dPosition(cv::Point2f position);
  void initialRegistration();
  bool isRegistrationDone();
  bool getRegistrationTransform(Eigen::Matrix4f& registration_transform);
  void projectIn2d(std::vector<cv::Point3f> points_3d, std::vector<cv::Point2f> &points_2d);
  void projectIn2d(cv::Point3f points_3d, cv::Point2f& points_2d);
  void run();
  void refineRegistration();
  float computeImageSparsity();
  void addRegistrationPoint();

  std::vector<cv::Point2f> registration_points_2d_;
  std::vector<cv::Point3f> registration_points_3d_;

  // Tool Position
  cv::Point3f position_3d_; // Meter Units
  cv::Point2f position_2d_; // Pixel Units

private:
  Eigen::Matrix3f toSkewSymMatrix(Eigen::Vector3f v);
  Eigen::MatrixXf computeImageJacobian(cv::Point3f v);
  Eigen::Matrix4f invertTransformation(Eigen::Matrix4f T);

  // Registration
  bool registration_done_;
  bool start_collect_points_;

  // Camera Extrinsics: Rotation and translation vectors
  cv::Mat rvec_, tvec_;

  // Registration parameters
  int num_registration_pts_;
  float max_registration_error_;     // Maximum accepted retroprojection error
  float avg_registration_error_;     // Maximum accepted AVERAGE retroprojection error

  // Refinement
  float convergence_rate_;
  float min_pts_sparsity_;   // Pixel Units
  float registration_weight_;

  // Camera instrinsics
  cv::Mat intrinsics_, distortion_;

  // Image information
  int image_width_, image_height_;

  std::chrono::steady_clock::time_point last_time_ = std::chrono::high_resolution_clock::now();

  // TODO erase
  std::vector<bool> registration_points_acquired_;

};


#endif // REGISTRATION_HPP
