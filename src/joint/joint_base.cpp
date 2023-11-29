#include <rkdl/joint/joint_base.h>

namespace rkdl
{
JointBase::JointBase(Name id, JointType type, Vector3& pos_p, Vector3& pos_s)
: name_(id), pos_p_(pos_p), pos_s_(pos_s), joint_type_(type)
{
    tree_type_ = Tree::Internal;
}

JointBase::~JointBase() {}
}
