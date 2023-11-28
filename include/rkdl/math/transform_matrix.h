#pragma once

#include <rkdl/math/type.h>

namespace rkdl
{
class TransformMatrix
{
public:
    TransformMatrix() {rotation_.setIdentity(); translation_.setZero();}
    TransformMatrix(Matrix3& R, Vector3& t): rotation_(R), translation_(t) {}

    TransformMatrix operator*(const TransformMatrix& other) const noexcept
    {
        TransformMatrix temp;
        temp.rotation_ = rotation_*other.rotation_;
        temp.translation_ = translation_ + rotation_*other.translation_;
        return temp;
    }

    Vector3 operator*(const Vector3& other) const noexcept
    {
        Vector3 temp;
        temp = translation_ + rotation_*other;
        return temp;
    }

    Matrix3 operator*(const Matrix3& other) const noexcept
    {
        Matrix3 temp;
        temp = rotation_*other;
        return temp;
    }

    Jacobian operator*(const Jacobian& other) const noexcept
    {
        Jacobian temp(3, other.cols());
        temp = rotation_*(other.colwise() + translation_);
        return temp;
    }

    void setZero() {
        rotation_.setZero();
        translation_.setZero();
    }

    void setIdentity()
    {
        rotation_.setIdentity();
        translation_.setZero();
    }

    TransformMatrix inverse()
    {
        TransformMatrix temp;
        temp.rotation_ = rotation_.transpose();
        temp.translation_ = -rotation_.transpose()*translation_;
        return temp;
    }

    void setRotationMatrix(const Scalar& v, const Vector3& axis)
    {
        rotation_ = Eigen::AngleAxis<Scalar>(v, axis).toRotationMatrix();
    }

    Matrix3 rotation_;
    Vector3 translation_;
};

}