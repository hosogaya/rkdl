#pragma once 

#include <rkdl/joint/joint_base.h>

namespace rkdl
{
class FixedJoint: public JointBase
{
public:
    FixedJoint(Name id, Vector3& fixed_position , Matrix3& rotation_matrix);
    ~FixedJoint();

private:
    
};
}