#pragma once 

#include <rkdl/math/math.h>
#include <rkdl/robot_model.h>
#include <rkdl/data.h>

namespace rkdl
{
class Kinematics
{
public: 
    static TransformMatrix transformMatrix(RobotModel& model, const Name& frame_name, const Vector& q);
    static Vector3 posFK(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p);
    static Matrix3 rotFK(RobotModel& model, const Name& frame_name, const Vector& q, const Matrix3& r=Matrix3::Identity());
    // calculate dT/dq
    static TransformMatrix differentialTransformMatrix(RobotModel& model, const Name& frame_name, const Name& joint_name, const Vector& q);
    static Jacobian jacobian(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p);

    static void updateKinematics(RobotModel& model);

    static bool error() {return error_;}
private:
    static bool error_;
};

bool Kinematics::error_ = false;
}