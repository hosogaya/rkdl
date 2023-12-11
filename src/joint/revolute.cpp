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

TransformMatrix RevoluteJoint::transformMatrix(const Scalar& p) const
{
    TransformMatrix tm;
    setTransformMatrix(tm, p);
    tm.translation_ = fixed_position_;
    return tm;
}

TransformMatrix RevoluteJoint::differentialOperator(const Scalar& p) const 
{
    return differential_operator_;
}

TransformMatrix RevoluteJoint::differentialTransformMatrix(const Scalar& p) const 
{
    return transformMatrix(p)*differentialOperator(p);
}

TransformMatrix RevoluteJoint::timeDifferentialOPerator(const Scalar& p, const Scalar& v) const
{
    if (velocity_ == v) return time_differential_operator_;
    else 
    {
        TransformMatrix tm;
        setTimeDifferentialOperator(tm, v);
        return tm;
    }
}

void RevoluteJoint::setTransformMatrix(TransformMatrix& tm, const Scalar& p) const
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
    }
}

void RevoluteJoint::setDifferentialOperator(TransformMatrix& tm, const Scalar& p) const
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
        tm.setRotationMatrix(M_PI_2, axis_vec_);
    }
}

void RevoluteJoint::setTimeDifferentialOperator(TransformMatrix& tm, const Scalar& v) const
{
    tm.setZero();
    if (axis_ == RevoluteAxis::X)
    {
        tm.rotation_(1,1) =-v;
        tm.rotation_(2,2) =-v;
    }
    else if (axis_ == RevoluteAxis::Y)
    {
        tm.rotation_(0,0) =-v;
        tm.rotation_(2,2) =-v;
    }
    else if (axis_ == RevoluteAxis::Z)
    {
        tm.rotation_(0,0) =-v;
        tm.rotation_(1,1) =-v;
    }
    else if (axis_ == RevoluteAxis::None)
    {
        tm.setRotationMatrix(M_PI, axis_vec_);
        tm.rotation_ *= v;
    }
}


void RevoluteJoint::setPosition(const Scalar& p) 
{
    position_ = p;
    setTransformMatrix(transform_matrix_, p);
}

void RevoluteJoint::setVelocity(const Scalar& v)
{
    velocity_ = v;
    setTimeDifferentialOperator(time_differential_operator_, velocity_);
}

void RevoluteJoint::setAccelration(const Scalar& a) {accel_ = a;}
void RevoluteJoint::setTorque(const Scalar& t) {torque_ = t;}


}