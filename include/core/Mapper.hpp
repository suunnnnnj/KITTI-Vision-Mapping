// include/core/Mapper.hpp
#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <liblas/liblas.hpp>
#include <Eigen/Core>
#include "io/ImageLoader.hpp"
#include "io/PoseLoader.hpp"
#include "utils/progress.hpp"

namespace kitti_mapping {
namespace core {

/**
 * @brief Core class to build a colored point map from KITTI data.
 */
class Mapper {
public:
    /**
     * @param imgDir     Directory containing color images.
     * @param imgPattern printf-style pattern for image filenames.
     * @param poseFile   Path to the pose file.
     * @param outputLAS  Output LAS filename.
     */
    Mapper(const std::string& imgDir,
           const std::string& imgPattern,
           const std::string& poseFile,
           const std::string& outputLAS);

    /** @brief Execute the mapping pipeline. */
    void run();

private:
    std::string imgDir_;
    std::string imgPattern_;
    std::string poseFile_;
    std::string outputLAS_;

    io::ImageLoader imgLoader_;
    io::PoseLoader poseLoader_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    utils::ProgressBar progressBar_;  ///< tracks progress over frames

    /**
     * @brief Projects depth points into camera frame and colors them.
     * @param image  Current color frame.
     * @param pose   4x4 transform from world to camera.
     */
    void processFrame(const cv::Mat& image, const Eigen::Matrix4f& pose);

    /** @brief Writes accumulated cloud to LAS file. */
    void saveCloud() const;

    /**
     * @brief Initializes loaders and point cloud container.
     * Called at beginning of run().
     */
    void initialize();
};

}  // namespace core
}  // namespace kitti_mapping

