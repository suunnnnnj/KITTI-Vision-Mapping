// include/utils/math.hpp
#pragma once

#include <Eigen/Core>

namespace kitti_mapping {
namespace utils {

/** @brief Convert Euler angles (roll, pitch, yaw) to rotation matrix. */
Eigen::Matrix3f eulerToRotation(float roll, float pitch, float yaw);

/** @brief Apply 4x4 transform to 3D point. */
Eigen::Vector3f transformPoint(const Eigen::Matrix4f& tf, const Eigen::Vector3f& pt);

}  // namespace utils
}  // namespace kitti_mapping

