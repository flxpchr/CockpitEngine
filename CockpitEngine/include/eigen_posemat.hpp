#ifndef EIGEN_POSEMAT_HPP
#define EIGEN_POSEMAT_HPP

#include <Eigen/Dense>
#include <iostream>

namespace Eigen{
class PoseMat : public Matrix4f
{
public:
    /// Constructors
    PoseMat():Matrix4f(Matrix4f::Identity()) {}
    template<typename OtherDerived>
    PoseMat(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix4f(other) {}
    PoseMat(const Matrix3f& rot, const Vector3f& trans) :
        Matrix4f(( Translation3f(trans)*Affine3f(rot)).matrix()) {}
    PoseMat(const float& roll, const float& pitch, const float& yaw, const Vector3f& trans) :
        Matrix4f(( Translation3f(trans)*
                   AngleAxisf(yaw, Eigen::Vector3f::UnitZ())*
                   AngleAxisf(pitch, Eigen::Vector3f::UnitY())*
                   AngleAxisf(roll, Eigen::Vector3f::UnitX())).matrix()) {}
    PoseMat(const float& roll, const float& pitch, const float& yaw) : PoseMat(roll, pitch, yaw, Vector3f::Zero()) {}

    /// Operators
    template<typename OtherDerived>
    PoseMat& operator=(const Eigen::MatrixBase <OtherDerived>& other){
        this->Eigen::Matrix4f::operator=(other);
        return *this;
    }
    template<typename OtherDerived>
    PoseMat operator*(const Eigen::MatrixBase <OtherDerived>& other){
        return this->Eigen::Matrix4f::operator*(other);
    }

    /// Referenced getters/setters
    Ref<Vector3f> trans(){
        return this->block<3,1>(0,3);
    }
    float& trans_x(){
        return this->coeffRef(0,3);
    }
    float& trans_y(){
        return this->coeffRef(1,3);
    }
    float& trans_z(){
        return this->coeffRef(2,3);
    }
    Ref<Matrix3f> rot(){
        return this->block<3,3>(0,0);
    }
    Ref<Vector3f> rot_x(){
        return this->block<3,1>(0,0);
    }
    Ref<Vector3f> rot_y(){
        return this->block<3,1>(0,1);
    }
    Ref<Vector3f> rot_z(){
        return this->block<3,1>(0,2);
    }

    /// Simple getters
    Quaternionf getQuaternion(){
        return Quaternionf(this->rot());
    }
    Vector3f getRPY(){
        return this->rot().eulerAngles(2,1,0).reverse();
    }
    float getRoll(){
        return this->getRPY()(0);
    }
    float getPitch(){
        return this->getRPY()(1);
    }
    float getYaw(){
        return this->getRPY()(2);
    }

    /// Setters
    void setRPY(const float& roll, const float& pitch, const float& yaw){
        this->rot() = Eigen::PoseMat(roll,pitch,yaw).rot();
    }
    void setRoll(const float& roll){
        this->setRPY(roll,this->getPitch(),this->getYaw());
    }
    void setPitch(const float& pitch){
        this->setRPY(this->getRoll(),pitch,this->getYaw());
    }
    void setYaw(const float& yaw){
        this->setRPY(this->getRoll(),this->getPitch(),yaw);
    }

    /// In-place transformations
    void transform(const Matrix4f& mat){
        (*this) *= Eigen::PoseMat(mat);
    }
    void transform(const float& roll, const float& pitch, const float& yaw, const Vector3f& trans){
        transform(Eigen::PoseMat(roll,pitch,yaw, trans));
    }
    void transform(const Matrix3f& rot, const Vector3f& trans){
        transform(Eigen::PoseMat(rot, trans));
    }
    void pretransform(const Matrix4f& mat){
        (*this) = Eigen::PoseMat(mat) *(*this);
    }
    void pretransform(const float& roll, const float& pitch, const float& yaw, const Vector3f& trans){
        pretransform(Eigen::PoseMat(roll,pitch,yaw, trans));
    }
    void pretransform(const Matrix3f& rot, const Vector3f& trans){
        pretransform(Eigen::PoseMat(rot, trans));
    }
    void rotate(const float& roll, const float& pitch, const float& yaw){
        this->transform(roll,pitch,yaw,Eigen::Vector3f::Zero());
    }
    void rotate(const Matrix3f& rot){
        this->transform(rot,Eigen::Vector3f::Zero());
    }
    void prerotate(const float& roll, const float& pitch, const float& yaw){
        this->pretransform(roll,pitch,yaw,Eigen::Vector3f::Zero());
    }
    void prerotate(const Matrix3f& rot){
        this->pretransform(rot,Eigen::Vector3f::Zero());
    }
    void translate(const Vector3f& trans){
        this->transform(0,0,0,trans);
    }
    void translate(const float& dx, const float& dy, const float& dz){
        this->translate(Eigen::Vector3f(dx,dy,dz));
    }
    void pretranslate(const Vector3f& trans){
        this->pretransform(0,0,0,trans);
    }
    void pretranslate(const float& dx, const float& dy, const float& dz){
        this->pretranslate(Eigen::Vector3f(dx,dy,dz));
    }

    /// Transformed copy
    Eigen::PoseMat transformed(const Matrix4f& mat){
        Eigen::PoseMat matcopy(*this);
        matcopy.transform(mat);
        return matcopy;
    }
    Eigen::PoseMat transformed(const float& roll, const float& pitch, const float& yaw, const Vector3f& trans){
        return transformed(Eigen::PoseMat(roll,pitch,yaw, trans));
    }
    Eigen::PoseMat transformed(const Matrix3f& rot, const Vector3f& trans){
        return transformed(Eigen::PoseMat(rot, trans));
    }
    Eigen::PoseMat pretransformed(const Matrix4f& mat){
        Eigen::PoseMat matcopy(*this);
        matcopy.pretransform(mat);
        return matcopy;
    }
    Eigen::PoseMat pretransformed(const float& roll, const float& pitch, const float& yaw, const Vector3f& trans){
        return pretransformed(Eigen::PoseMat(roll,pitch,yaw, trans));
    }
    Eigen::PoseMat pretransformed(const Matrix3f& rot, const Vector3f& trans){
        return pretransformed(Eigen::PoseMat(rot, trans));
    }
    Eigen::PoseMat rotated(const float& roll, const float& pitch, const float& yaw){
        Eigen::PoseMat matcopy(*this);
        matcopy.rotate(roll,pitch,yaw);
        return matcopy;
    }
    Eigen::PoseMat rotated(const Matrix3f& rot){
        Eigen::PoseMat matcopy(*this);
        matcopy.rotate(rot);
        return matcopy;
    }
    Eigen::PoseMat prerotated(const float& roll, const float& pitch, const float& yaw){
        Eigen::PoseMat matcopy(*this);
        matcopy.prerotate(roll,pitch,yaw);
        return matcopy;
    }
    Eigen::PoseMat prerotated(const Matrix3f& rot){
        Eigen::PoseMat matcopy(*this);
        matcopy.prerotate(rot);
        return matcopy;
    }
    Eigen::PoseMat translated(const Vector3f& trans){
        Eigen::PoseMat matcopy(*this);
        matcopy.translate(trans);
        return matcopy;
    }
    Eigen::PoseMat translated(const float& dx, const float& dy, const float& dz){
        return this->translated(Eigen::Vector3f(dx,dy,dz));
    }
    Eigen::PoseMat pretranslated(const Vector3f& trans){
        Eigen::PoseMat matcopy(*this);
        matcopy.pretranslate(trans);
        return matcopy;
    }
    Eigen::PoseMat pretranslated(const float& dx, const float& dy, const float& dz){
        return this->pretranslated(Eigen::Vector3f(dx,dy,dz));
    }
};
}

#endif // EIGEN_POSEMAT_HPP
