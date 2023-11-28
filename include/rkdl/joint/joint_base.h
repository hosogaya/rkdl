#pragma once 

#include <rkdl/math/math.h>
#include <iostream>

namespace rkdl
{
typedef enum
{
    Fixed=0,
    Revolute
} JointType;


class JointBase
{
public:
    JointBase(Name name_, JointType type, Vector3& pos_p, Vector3& pos_s);
    ~JointBase();

    bool isLeaf() {return child_joint_index_ < 0;}
    bool isRoot() {return parent_joint_index_ < 0;}

    virtual const TransformMatrix& transformMatrix() const {return transform_matrix_;};
    virtual const TransformMatrix& differentialOperator() const {return differential_operator_;}

    virtual void setPosition(const Scalar p) {}
    virtual void setVelocity(const Scalar v) {}
    virtual void setAccelration(const Scalar a) {}
    virtual void setForce(const Scalar f) {}

    virtual void print() {std::cout << "JointBase" << std::endl;}

    const Name name_;
    // Name parent_joint_;
    // Name child_joint_;
    // Name parent_frame_;
    int child_joint_index_;
    int parent_joint_index_;
     // fixed position relative to successor body origin (pos_s_) and prodecessor body origin (pos_p_)
    const Vector3 pos_s_;
    const Vector3 pos_p_;
    const JointType joint_type_;

protected:
    // transform matrix
    TransformMatrix transform_matrix_;
    TransformMatrix differential_operator_;
    
};
}