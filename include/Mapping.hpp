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
            
            std::string lidarPath, imagePath, timesPath, posePath, calibPath;

            lidarPath = dataPath + "/data_odometry_velodyne/dataset/" + zeroPadding(sequence, 2) + "/velodyne";
            imagePath = dataPath + "/data_odometry_gray/dataset/sequences/" + zeroPadding(sequence, 2) + "/image_1"; // Need to change;
            timesPath = dataPath + "/data_odometry_calib/dataset/sequences/"  + zeroPadding(sequence, 2) + "/times.txt";;

            posePath  = dataPath + "/data_odometry_poses/dataset/poses/"  + zeroPadding(sequence, 2) + ".txt";
            calibPath = dataPath + "/data_odometry_calib/dataset/sequences/"  + zeroPadding(sequence, 2) + "/calib.txt";

            std::cout << "-------------------------------" << std::endl;
            std::cout << "lidarPath is - " << lidarPath << std::endl;
            std::cout << "imagePath is - " << imagePath << std::endl;
            std::cout << "posePath is  - " << posePath  << std::endl;
            std::cout << "calibpath is - " << calibPath << std::endl;

            lidar_ = listdir(lidarPath, ".bin");
            image_ = listdir(imagePath, ".png");

            std::cout << "number of lidar : "  << lidar_.size() << std::endl;
            std::cout << "number of image : "  << image_.size() << std::endl;


        };

        KITTI_MAPPING(){

        };


    private:

        struct calib
        {
            cv::Mat Intrinsic;
            cv::Mat Extrinsic_R;
            cv::Mat Extrinsic_T;
            cv::Mat K;
        };
        
        std::vector<std::string> lidar_, image_, times_;

        calib calib_;

        std::vector<std::string> loadTimesData(const std::string &timesPath);
        std::vector<std::string> loadPosesData(const std::string &posePath);
        std::vector<std::string> loadCalibData(const std::string &calibPath);
};

#endif