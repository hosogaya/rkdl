#pragma once 
#define PURE_EIGEN

#ifndef PURE_EIGEN
#include <ArduinoEigen.h>
#else
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#endif

#include <string>

namespace rkdl
{
using Name = std::string;
using Scalar = float;
using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Jacobian = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

typedef enum
{
    Root=0,
    Internal,
    Leaf
} Tree;

}