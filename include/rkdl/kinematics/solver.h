#pragma once 

#include <rkdl/math/math.h>
#include <rkdl/robot_model.h>
#include <rkdl/data.h>

namespace rkdl
{
class Kinematics
{
public: 
    static TransformMatrix calTransformMatrix(const RobotModel& model, const Name& frame_name, const Vector& q);
    static Vector3 calPosFK(const RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p=Vector3::Zero());
    static Matrix3 calRotFK(const RobotModel& model, const Name& frame_name, const Vector& q, const Matrix3& r=Matrix3::Identity());
    // calculate dT/dq
    static TransformMatrix calDifferentialTransformMatrix(const RobotModel& model, const Name& frame_name, const Name& joint_name, const Vector& q);
    static Jacobian calJacobian(const RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p=Vector3::Zero());

    static void updateKinematics(RobotModel& model);
    static Vector3 fk(const RobotModel& model, const Name& frame_name, const Vector3& p=Vector3::Zero());
    static Jacobian jacobian(const RobotModel& model, const Name& frame_name, const Vector3& p=Vector3::Zero());
    static Jacobian jacobian_dot(const RobotModel& model, const Name& frame_name, const Vector3& p=Vector3::Zero());
    
    // https://www.jstage.jst.go.jp/article/jrsj/34/3/34_34_167/_pdf
    // https://www.jstage.jst.go.jp/article/jrsj/29/3/29_3_269/_pdf/-char/en
    // simple LM method with regularization term
    static bool ikPos(const RobotModel& model, const Name& frame_name, const Vector3& ref_x, Vector& q);

    static bool error() {return error_;}
    static Scalar ik_eva_thres_;
    static Scalar ik_eva_diff_thres_;
    static Scalar ik_diff_q_thres_;
    static Scalar ik_regularization_term_;
private:
    static bool error_;
};
}