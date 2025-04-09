// src/utils/fs.cpp
#include "utils/fs.hpp"
#include <filesystem>

namespace kitti_mapping {
namespace utils {

std::vector<std::string> listFiles(const std::string& directory, const std::string& ext) {
    std::vector<std::string> files;
    for (auto& entry : std::filesystem::directory_iterator(directory)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() == ext)
            files.push_back(entry.path().filename().string());
    }
    std::sort(files.begin(), files.end());
    return files;
}

void ensureDirectory(const std::string& path) {
    std::filesystem::create_directories(path);
}

}  // namespace utils
}  // namespace kitti_mapping

