#include <rkdl/joint/fixed.h>

namespace rkdl
{
FixedJoint::FixedJoint(Name id, Vector3& fixed_position, Matrix3& rotation_matrix)
:JointBase(id, JointType::Fixed, fixed_position)
{
    // set transform matrix
    transform_matrix_.topLeftCorner(3,3) = rotation_matrix;
    transform_matrix_.topRightCorner(3,1) = fixed_position_;

    differential_operator_.setZero();
}

FixedJoint::~FixedJoint() {}
}