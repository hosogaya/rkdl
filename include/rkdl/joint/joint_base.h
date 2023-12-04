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
    JointBase(Name name_, JointType type, Vector3& fixed_position);
    ~JointBase();

    bool isLeaf() {return tree_type_ == Tree::Leaf;}
    bool isRoot() {return tree_type_ == Tree::Root;}

    virtual const TransformMatrix& transformMatrix() const {return transform_matrix_;};
    virtual TransformMatrix transformMatrix(const Scalar& p) const {return transform_matrix_;}
    virtual const TransformMatrix& differentialOperator() const {return differential_operator_;}
    virtual TransformMatrix differentialOperator(const Scalar& p) const {return differential_operator_;}
    virtual TransformMatrix differentialTransformMatrix() const {return transform_matrix_*differential_operator_;}
    virtual TransformMatrix differentialTransformMatrix(const Scalar& p) const {return transform_matrix_*differential_operator_;}

    virtual void setPosition(const Scalar& p) {}
    virtual void setVelocity(const Scalar& v) {}
    virtual void setAccelration(const Scalar& a) {}
    virtual void setTorque(const Scalar& t) {}

    virtual void print() {std::cout << "JointBase" << std::endl;}
    void printName() {std::cout << name_ << std::endl;}

    const Name name_;
    // Name parent_joint_;
    // Name child_joint_;
    // Name parent_frame_;
    int child_joint_index_;
    int parent_joint_index_;
    int cajn_; // the number of child acutated joint
    int pajn_; // the number of parent acuated joint
     // fixed position relative to successor body origin (pos_s_) and prodecessor body origin (fixed_position_)
    // const Vector3 pos_s_;
    const Vector3 fixed_position_;
    const JointType joint_type_;
    Tree tree_type_;

protected:
    // transform matrix
    TransformMatrix transform_matrix_;
    TransformMatrix differential_operator_;
    
};
}