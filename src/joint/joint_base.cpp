#include <rkdl/joint/joint_base.h>

namespace rkdl
{
JointBase::JointBase(Name id, JointType type, Vector3& fixed_position)
: name_(id), fixed_position_(fixed_position), joint_type_(type)
{
    tree_type_ = Tree::Internal;
}

JointBase::~JointBase() {}
}
