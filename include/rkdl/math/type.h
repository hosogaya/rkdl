#pragma once 

#ifdef Arduino
#include <ArduinoEigen.h>
#else
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#endif

#include <string>
#include <unordered_map>

namespace rkdl
{
using Name = std::string;
using Scalar = float;
using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using Jacobian = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
using ActuatedJointMap = std::unordered_map<Name, Scalar>;
using FootMap = std::unordered_map<rkdl::Name, rkdl::Vector3>;
using Quaternion = Eigen::Quaternion<Scalar>;
typedef enum
{
    Root=0,
    Internal,
    Leaf
} Tree;

}