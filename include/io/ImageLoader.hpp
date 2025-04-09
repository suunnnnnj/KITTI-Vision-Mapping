// include/io/ImageLoader.hpp
#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace kitti_mapping {
namespace io {

/**
 * @brief Loads a sequence of color images from a directory.
 */
class ImageLoader {
public:
    /**
     * @param directory Path to the folder containing image files.
     * @param pattern   Filename pattern, e.g. "%06d.png".
     */
    ImageLoader(const std::string& directory, const std::string& pattern);

    /**
     * @brief Read next image in sequence.
     * @return CV Mat of the image; empty Mat if end of sequence.
     */
    cv::Mat next();

    /**
     * @brief Reset internal counter to start.
     */
    void reset();

    /**
     * @brief Total number of images detected.
     */
    size_t size() const;

private:
    std::string directory_;
    std::string pattern_;
    size_t index_;
    size_t total_;
    std::vector<std::string> file_list_;

    void scanDirectory();  // gathers file_list_
};

}  // namespace io
}  // namespace kitti_mapping

