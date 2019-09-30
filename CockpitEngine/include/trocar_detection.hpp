#ifndef TROCAR_DETECTION_HPP
#define TROCAR_DETECTION_HPP

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <robot_kinematics.hpp>
#include <instrument.hpp>

namespace trocar {
    static constexpr int        BUFF_SIZE = 80;
    static constexpr float     MIN_LINDIST = 0.005f; // m
    static constexpr float     MIN_PF_DIST = 0.08f; // m
    static constexpr float     MAX_ANGDIST = 0.99f; // cos(degree) : 0.99 from 5° to 90°
    static constexpr float     AVG_ERROR_THRESHOLD = 0.02f; // m
    static constexpr int        MAX_NB_STEP_NO_TROCAR = 10;
}

class TrocarDetection
{
public:
    TrocarDetection(RobotKinematics* kinematics, Instrument* instrument);
    ~TrocarDetection();

    Eigen::Vector3f getTrocarPosition();
    bool isTrocarFound();
    void update();

private:
    void computeTrocar();
    float computeError(const Eigen::Vector3f &estimatedTrocar);
    bool isDifferentEnough();

public:
    Eigen::Vector3f trocar_position_;
    bool is_trocar_found_;
    float FT_, PF_, GF_, BF_;

private:
    boost::circular_buffer<Eigen::Matrix<float,3,3,Eigen::RowMajor>> A_buff_{trocar::BUFF_SIZE};
    boost::circular_buffer<Eigen::Matrix<float,1,3>> b_buff_{trocar::BUFF_SIZE};
    boost::circular_buffer<Eigen::Vector3f> pose_buff_{trocar::BUFF_SIZE};
    boost::circular_buffer<Eigen::Vector3f> z_axis_buff_{trocar::BUFF_SIZE};

    Eigen::Map<Eigen::Matrix<float,trocar::BUFF_SIZE*3,3,Eigen::RowMajor>> A_;
    Eigen::Map<Eigen::Matrix<float,trocar::BUFF_SIZE*3,1>> b_;

    int nb_step_no_trocar_;

    RobotKinematics* kinematics_;
    Instrument* instrument_;
};

#endif // TROCAR_DETECTION_HPP
