#pragma once

#define NAN_D std::nan("0")
#define NAN_V3D Eigen::Vector3d::Ones()*NAN_D
#define NAN_M3D Eigen::Matrix3d::Ones()*NAN_D