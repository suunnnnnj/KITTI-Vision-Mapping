// src/io/ImageLoader.cpp
#include "io/ImageLoader.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace kitti_mapping {
namespace io {

ImageLoader::ImageLoader(const std::string& directory, const std::string& pattern)
  : directory_(directory), pattern_(pattern), index_(0) {
    scanDirectory();
    total_ = file_list_.size();
}

void ImageLoader::scanDirectory() {
    for (const auto& entry : std::filesystem::directory_iterator(directory_)) {
        if (!entry.is_regular_file()) continue;
        file_list_.push_back(entry.path().filename().string());
    }
    std::sort(file_list_.begin(), file_list_.end());
}

cv::Mat ImageLoader::next() {
    if (index_ >= total_) return {};
    std::string filename = file_list_[index_++];
    return cv::imread(directory_ + "/" + filename, cv::IMREAD_COLOR);
}

void ImageLoader::reset() {
    index_ = 0;
}

size_t ImageLoader::size() const {
    return total_;
}

}  // namespace io
}  // namespace kitti_mapping

