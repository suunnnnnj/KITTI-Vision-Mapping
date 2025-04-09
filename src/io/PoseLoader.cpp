// src/io/PoseLoader.cpp
#include "io/PoseLoader.hpp"
#include <fstream>
#include <sstream>

namespace kitti_mapping {
namespace io {

PoseLoader::PoseLoader(const std::string& filepath)
  : filepath_(filepath), index_(0) {
    loadFile();
}

void PoseLoader::loadFile() {
    std::ifstream ifs(filepath_);
    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        // file has 12 values per line: row-major 3x4 matrix
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
                iss >> M(i, j);
        poses_.push_back(M);
    }
}

Eigen::Matrix4f PoseLoader::next() {
    if (index_ >= poses_.size()) return Eigen::Matrix4f::Identity();
    return poses_[index_++];
}

void PoseLoader::reset() {
    index_ = 0;
}

size_t PoseLoader::size() const {
    return poses_.size();
}

}  // namespace io
}  // namespace kitti_mapping

