// src/utils/progress.cpp
#include "utils/progress.hpp"
#include <iostream>
#include <iomanip>

namespace kitti_mapping {
namespace utils {

ProgressBar::ProgressBar(size_t total, size_t width)
  : total_(total), width_(width), current_(0),
    startTime_(std::chrono::steady_clock::now()) {}

void ProgressBar::tick() {
    ++current_;
    display();
}

void ProgressBar::display() {
    float ratio = static_cast<float>(current_) / total_;
    size_t filled = static_cast<size_t>(ratio * width_);
    std::cout << "\r[";
    for (size_t i = 0; i < filled; ++i) std::cout << '=';
    for (size_t i = filled; i < width_; ++i) std::cout << ' ';
    std::cout << "] " << std::setw(3) << int(ratio * 100) << "%";
    std::cout.flush();
}

void ProgressBar::finish() {
    current_ = total_;
    display();
    std::cout << std::endl;
}

}  // namespace utils
}  // namespace kitti_mapping

