#include <rkdl/joint/fixed.h>

namespace rkdl
{
FixedJoint::FixedJoint(Name id, Vector3& pos_p, Vector3& pos_s, Matrix3& rotation_matrix)
:JointBase(id, JointType::Fixed, pos_p, pos_s)
{
    // set transform matrix
    transform_matrix_.rotation_ = rotation_matrix;
    transform_matrix_.translation_ = rotation_matrix*pos_s_ + pos_p_;

    differential_operator_.setZero();
}

FixedJoint::~FixedJoint() {}
}