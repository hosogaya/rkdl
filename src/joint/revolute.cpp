#include <rkdl/joint/revolute.h>

namespace rkdl
{
RevoluteJoint::RevoluteJoint(Name id, Vector3& fixed_position, Vector3& axis_vec)
: JointBase(id, JointType::Revolute, fixed_position), axis_vec_(axis_vec), axis_(RevoluteAxis::None)
{
    transform_matrix_.rotation_.setIdentity();
    transform_matrix_.translation_ = fixed_position;

    setDifferentialOperator(differential_operator_);
}

RevoluteJoint::RevoluteJoint(Name id, Vector3& fixed_position, RevoluteAxis axis)
: JointBase(id, JointType::Revolute, fixed_position), axis_(axis)
{
    transform_matrix_.rotation_.setIdentity();
    transform_matrix_.translation_ = fixed_position;
    setDifferentialOperator(differential_operator_);
}

RevoluteJoint::~RevoluteJoint() {}

TransformMatrix RevoluteJoint::transformMatrix(const Scalar p) const
{
    TransformMatrix tm;
    setTransformMatrix(tm, p);
    return tm;
}

TransformMatrix RevoluteJoint::differentialOperator(const Scalar p) const 
{
    if (axis_ == RevoluteAxis::None) {
        TransformMatrix tm;
        setDifferentialOperator(tm, p);
        return tm;
    }
    return differential_operator_;
}


TransformMatrix RevoluteJoint::differentialTransformMatrix(const Scalar p) const 
{
    return transformMatrix(p)*differentialOperator(p);
}

void RevoluteJoint::setTransformMatrix(TransformMatrix& tm, const Scalar p) const
{
    if (axis_ == RevoluteAxis::X)
    {
        tm.rotation_(0,0) = 1.0;
        tm.rotation_(1,1) = std::cos(p);
        tm.rotation_(1,2) = -std::sin(p);
        tm.rotation_(2,1) = -tm.rotation_(1,2);
        tm.rotation_(2,2) = tm.rotation_(1,1);
    }
    else if (axis_ == RevoluteAxis::Y)
    {
        tm.rotation_(0,0) = std::cos(p);
        tm.rotation_(0,2) = std::sin(p);
        tm.rotation_(1,1) = 1.0;
        tm.rotation_(2,0) = -tm.rotation_(0,2);
        tm.rotation_(2,2) = tm.rotation_(0,0);
    }
    else if (axis_ == RevoluteAxis::Z)
    {
        tm.rotation_(0,0) = std::cos(p);
        tm.rotation_(0,1) = -std::sin(p);
        tm.rotation_(1,0) = -tm.rotation_(0,1);
        tm.rotation_(1,1) = tm.rotation_(0,0);
        tm.rotation_(2,2) = 1.0;
    }
    else if (axis_ == RevoluteAxis::None)
    {
        /**
         * if axis = uint_z
         * rot << c,-s, 0,
         *        s, c, 0,
         *        0, 0, 1
        */
        tm.setRotationMatrix(p, axis_vec_);
        tm.translation_ = fixed_position_;   
    }
}

void RevoluteJoint::setDifferentialOperator(TransformMatrix& tm, const Scalar p) const
{
    tm.setZero();
    if (axis_ == RevoluteAxis::X)
    {
        tm.rotation_(1,2) =-1.0;
        tm.rotation_(2,1) = 1.0;
    }
    else if (axis_ == RevoluteAxis::Y)
    {
        tm.rotation_(0,2) = 1.0;
        tm.rotation_(2,0) =-1.0;
    }
    else if (axis_ == RevoluteAxis::Z)
    {
        tm.rotation_(0,1) =-1.0;
        tm.rotation_(1,0) = 1.0;
    }
    else if (axis_ == RevoluteAxis::None)
    {
        tm.setRotationMatrix(p+M_PI_2, axis_vec_);
    }
}

void RevoluteJoint::setPosition(Scalar p)
{
    position_ = p;
    setTransformMatrix(transform_matrix_, p);
}

void RevoluteJoint::setVelocity(Scalar v)
{
    velocity_ = v;
}

void RevoluteJoint::setAccelration(Scalar a) {accel_ = a;}
void RevoluteJoint::setTorque(Scalar t) {torque_ = t;}


}