#include <rkdl/joint/revolute.h>

namespace rkdl
{
RevoluteJoint::RevoluteJoint(Name id, Vector3& pos_p, Vector3& pos_s, Vector3& axis_vec)
: JointBase(id, JointType::Revolute, pos_p, pos_s), axis_vec_(axis_vec), axis_(RevoluteAxis::None)
{
    transform_matrix_.rotation_.setIdentity();
    transform_matrix_.translation_ = pos_p + pos_s;

    setDifferentialOperator();
}

RevoluteJoint::RevoluteJoint(Name id, Vector3& pos_p, Vector3& pos_s, RevoluteAxis axis)
: JointBase(id, JointType::Revolute, pos_p, pos_s), axis_(axis)
{
    transform_matrix_.rotation_.setIdentity();
    transform_matrix_.translation_ = pos_p + pos_s;
    setDifferentialOperator();
}

RevoluteJoint::~RevoluteJoint() {}

void RevoluteJoint::setDifferentialOperator()
{
    differential_operator_.setZero();
    if (axis_ == RevoluteAxis::X)
    {
        differential_operator_.rotation_(1,2) =-1.0;
        differential_operator_.rotation_(2,1) = 1.0;
    }
    else if (axis_ == RevoluteAxis::Y)
    {
        differential_operator_.rotation_(0,2) = 1.0;
        differential_operator_.rotation_(2,0) =-1.0;
    }
    else if (axis_ == RevoluteAxis::Z)
    {
        differential_operator_.rotation_(0,1) =-1.0;
        differential_operator_.rotation_(1,0) = 1.0;
    }
    else if (axis_ == RevoluteAxis::None)
    {
        // Todo
    }
}

void RevoluteJoint::setPosition(Scalar p)
{
    position_ = p;
    if (axis_ == RevoluteAxis::X)
    {
        transform_matrix_.rotation_(0,0) = 1.0;
        transform_matrix_.rotation_(1,1) = std::cos(p);
        transform_matrix_.rotation_(1,2) = -std::sin(p);
        transform_matrix_.rotation_(2,1) = -transform_matrix_.rotation_(1,2);
        transform_matrix_.rotation_(2,2) = transform_matrix_.rotation_(1,1);
    }
    else if (axis_ == RevoluteAxis::Y)
    {
        transform_matrix_.rotation_(0,0) = std::cos(p);
        transform_matrix_.rotation_(0,2) = std::sin(p);
        transform_matrix_.rotation_(1,1) = 1.0;
        transform_matrix_.rotation_(2,0) = -transform_matrix_.rotation_(0,2);
        transform_matrix_.rotation_(2,2) = transform_matrix_.rotation_(0,0);
    }
    else if (axis_ == RevoluteAxis::Z)
    {
        transform_matrix_.rotation_(0,0) = std::cos(p);
        transform_matrix_.rotation_(0,1) = -std::sin(p);
        transform_matrix_.rotation_(1,0) = -transform_matrix_.rotation_(0,1);
        transform_matrix_.rotation_(1,1) = transform_matrix_.rotation_(0,0);
        transform_matrix_.rotation_(2,2) = 1.0;
    }
    else if (axis_ == RevoluteAxis::None)
    {
        /**
         * if axis = uint_z
         * rot << c,-s, 0,
         *        s, c, 0,
         *        0, 0, 1
        */
        transform_matrix_.setRotationMatrix(p, axis_vec_);
        transform_matrix_.translation_ = transform_matrix_.rotation_*pos_s_ + pos_p_;   
    }
}

void RevoluteJoint::setVelocity(Scalar v)
{
    velocity_ = v;
}

void RevoluteJoint::setAccelration(Scalar a) {accel_ = a;}
void RevoluteJoint::setForce(Scalar f) {torque_ = f;}


}