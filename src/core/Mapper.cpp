// src/core/Mapper.cpp
#include "core/Mapper.hpp"
#include <pcl/io/las_io.h>  // for LAS writing
#include <iostream>

namespace kitti_mapping {
namespace core {

Mapper::Mapper(const std::string& imgDir,
               const std::string& imgPattern,
               const std::string& poseFile,
               const std::string& outputLAS)
  : imgDir_(imgDir),
    imgPattern_(imgPattern),
    poseFile_(poseFile),
    outputLAS_(outputLAS),
    imgLoader_(imgDir, imgPattern),
    poseLoader_(poseFile),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
    progressBar_(imgLoader_.size())
{}

void Mapper::initialize() {
    // pre-allocate rough size to avoid reallocations
    cloud_->reserve(imgLoader_.size() * 1000);  
    poseLoader_.reset();
    imgLoader_.reset();
}

void Mapper::run() {
    initialize();
    while (true) {
        auto img = imgLoader_.next();
        if (img.empty()) break;
        auto pose = poseLoader_.next();
        processFrame(img, pose);
        progressBar_.tick();
    }
    progressBar_.finish();
    saveCloud();
}

void Mapper::processFrame(const cv::Mat& image, const Eigen::Matrix4f& pose) {
    // assume depth encoded in blue channel; project each pixel to world
    const float fx = 721.5377f, fy = 721.5377f;
    const float cx = 609.5593f, cy = 172.8540f;
    for (int v = 0; v < image.rows; ++v) {
        for (int u = 0; u < image.cols; ++u) {
            uint8_t depth = image.at<cv::Vec3b>(v,u)[0];
            if (depth == 0) continue;
            Eigen::Vector3f camPt(
                (u - cx) * depth / fx,
                (v - cy) * depth / fy,
                depth
            );
            Eigen::Vector3f worldPt = pose * camPt.homogeneous();
            pcl::PointXYZRGB pt;
            pt.x = worldPt.x(); pt.y = worldPt.y(); pt.z = worldPt.z();
            auto color = image.at<cv::Vec3b>(v,u);
            pt.r = color[2]; pt.g = color[1]; pt.b = color[0];
            cloud_->push_back(pt);
        }
    }
}

void Mapper::saveCloud() const {
    pcl::io::LASWriter writer;
    writer.write(outputLAS_, *cloud_);
}

}  // namespace core
}  // namespace kitti_mapping

