#include <instrument.hpp>

Instrument::Instrument(const float BP, const float PG, const float GT, const float mass){
    m_BP = BP;
    m_PG = PG;
    m_GT = GT;
    m_BT = m_BP + m_PG+ m_GT;
    m_PT = m_PG + m_GT;
    m_mass = mass;

    m_tipCalibration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_axisOrigin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_axisDirection = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_shaftRadius = 0.0f;
    m_tipRadius = 0.0f;
}














