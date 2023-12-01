#pragma once 

#include <rkdl/math/math.h>

namespace rkdl
{

class Frame
{
public:
    Frame(Name id, Name parent_frame, Name parent_joint, Scalar m, Vector3& cog);
    ~Frame();

    bool isRoot() {return type_==Tree::Root;}
    bool isLeaf() {return type_==Tree::Leaf;}

    const Name name_;
    const Name parent_frame_;
    const Name parent_joint_;

    int parent_frame_index_;
    int parent_joint_index_;

    Tree type_;
    TransformMatrix transform_matirx_;

    const Scalar mass_;
    const Vector3 cog_;

};

}