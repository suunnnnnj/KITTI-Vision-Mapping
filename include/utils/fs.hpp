// include/utils/fs.hpp
#pragma once

#include <string>
#include <vector>

namespace kitti_mapping {
namespace utils {

/** @brief List files in directory matching extension (e.g. ".png"). */
std::vector<std::string> listFiles(const std::string& directory, const std::string& ext);

/** @brief Create directory if it doesn't exist. */
void ensureDirectory(const std::string& path);

}  // namespace utils
}  // namespace kitti_mapping

