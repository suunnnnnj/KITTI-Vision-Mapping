// include/io/PoseLoader.hpp
#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

namespace kitti_mapping {
namespace io {

/**
 * @brief Parses pose (GPS/IMU) data from text file.
 */
class PoseLoader {
public:
    /**
     * @param filepath Path to the poses file (ASCII, one 3x4 matrix per line).
     */
    explicit PoseLoader(const std::string& filepath);

    /**
     * @brief Get next pose as 4x4 transformation matrix.
     * @return Eigen::Matrix4f; identity if no more entries.
     */
    Eigen::Matrix4f next();

    /**
     * @brief Reset to first pose.
     */
    void reset();

    /**
     * @brief Number of poses loaded.
     */
    size_t size() const;

private:
    std::string filepath_;
    size_t index_;
    std::vector<Eigen::Matrix4f> poses_;

    void loadFile();  // fills poses_
};

}  // namespace io
}  // namespace kitti_mapping

