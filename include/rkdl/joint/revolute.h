#pragma once

#include <rkdl/joint/joint_base.h>

namespace rkdl
{
typedef enum
{
    None=0,
    X,
    Y,
    Z
} RevoluteAxis;

class RevoluteJoint: public JointBase
{
public: 
    RevoluteJoint(Name id, Vector3& pos_p, Vector3& pos_s, Vector3& axis_vec);
    RevoluteJoint(Name id, Vector3& pos_p, Vector3& pos_s, RevoluteAxis axis);
    ~RevoluteJoint();

    void print() override {std::cout << "RevoluteJoint" << std::endl;}
    TransformMatrix transformMatrix(const Scalar p) const override;
    TransformMatrix differentialTransformMatrix(const Scalar p) const override;
    TransformMatrix differentialOperator(const Scalar p) const override;


    virtual void setPosition(const Scalar p) override;
    virtual void setVelocity(const Scalar v) override;
    virtual void setAccelration(const Scalar a) override;
    virtual void setTorque(const Scalar t) override;

    const Vector3 axis_vec_;    
    const RevoluteAxis axis_;
protected:
    Scalar position_;
    Scalar velocity_;
    Scalar accel_;
    Scalar torque_;

    void setTransformMatrix(TransformMatrix& tm, const Scalar p) const;
    void setDifferentialOperator(TransformMatrix& tm, const Scalar p=0.0) const;
};
}