// src/utils/math.cpp
#include "utils/math.hpp"
#include <cmath>

namespace kitti_mapping {
namespace utils {

Eigen::Matrix3f eulerToRotation(float roll, float pitch, float yaw) {
    Eigen::AngleAxisf R_x(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf R_y(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf R_z(yaw,   Eigen::Vector3f::UnitZ());
    return (R_z * R_y * R_x).toRotationMatrix();
}

Eigen::Vector3f transformPoint(const Eigen::Matrix4f& tf, const Eigen::Vector3f& pt) {
    Eigen::Vector4f homo(pt.x(), pt.y(), pt.z(), 1.0f);
    Eigen::Vector4f res = tf * homo;
    return res.head<3>();
}

}  // namespace utils
}  // namespace kitti_mapping

