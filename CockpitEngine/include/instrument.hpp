#ifndef INSTRUMENT_HPP
#define INSTRUMENT_HPP

#include <stdio.h>
#include <iostream>

#include <Eigen/Dense>


// G = center of gravity
// P = robot end-effector position (position of magnet along the shaft)
// T = instrument tip
// B = instrument base (opposite to the tip)
// mass = total mass of the instrument in Kg

class Instrument{

public:
    Instrument(const float BP, const float PG, const float GT, const float mass);

    Eigen::Vector3f getTipCalibration(){return m_tipCalibration;}
    Eigen::Vector3f getCalibAxisPosition(){return m_axisOrigin;}
    Eigen::Vector3f getCalibAxisDirection(){return m_axisDirection;}

    float m_tipRadius;
    float m_shaftRadius;
    float m_mass;
    float m_BP, m_PG, m_GT;
    float m_PT, m_BT;

private:
    Eigen::Vector3f m_tipCalibration;
    Eigen::Vector3f m_axisOrigin;
    Eigen::Vector3f m_axisDirection;
};

#endif // INSTRUMENT_HPP
