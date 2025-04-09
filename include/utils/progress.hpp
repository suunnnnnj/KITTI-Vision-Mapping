// include/utils/progress.hpp
#pragma once

#include <string>
#include <chrono>

namespace kitti_mapping {
namespace utils {

/**
 * @brief Simple console progress bar.
 *
 * Only supports incremental updates; does not rewind.
 */
class ProgressBar {
public:
    /**
     * @param total   Total count to reach 100%.
     * @param width   Width of the bar in characters.
     */
    ProgressBar(size_t total, size_t width = 50);

    /** @brief Advance the bar by one step. */
    void tick();

    /** @brief Finalize and move to next line. */
    void finish();

private:
    size_t total_;
    size_t width_;
    size_t current_;
    std::chrono::time_point<std::chrono::steady_clock> startTime_;

    void display();  // redraws bar
};

}  // namespace utils
}  // namespace kitti_mapping

