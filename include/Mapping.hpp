#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "utils.hpp"

#include "iostream"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

class KITTI_MAPPING
{
    public:

        KITTI_MAPPING(const std::string &dataPath, const int &sequence){
            
            std::string lidarPath;
            std::string imagePath; // using only Color1 image;

            // lidarPath = dataPath + "/data_odometry_velodyne/dataset/sequences/" + zeroPadding(sequence, 2) + "/velodyne";
            // imagePath = dataPath + "/data_odometry_gray/dataset/sequences/" + zeroPadding(sequence, 2) + "/image_2";

            // std::cout << "lidarPath is - " << lidarPath << std::endl;
            // std::cout << "imagePath is - " << imagePath << std::endl;


        };

        KITTI_MAPPING(){

        };


    private:

        std::vector<std::string> lidar_, image_;
        std::string posePath_, calibPath_;
};

#endif