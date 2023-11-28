#pragma once 

#include <rkdl/joint/joint_base.h>

namespace rkdl
{
class FixedJoint: public JointBase
{
public:
    FixedJoint(Name id, Vector3& pos_p, Vector3& pos_s, Matrix3& rotation_matrix);
    ~FixedJoint();

    void print() override {std::cout << "FixedJoint" << std::endl;}

private:
    
};
}