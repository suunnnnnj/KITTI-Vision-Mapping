#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "utils.hpp"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

class KITTI_MAPPING
{
    public:

        KITTI_MAPPING(const std::string &dataPath, const int &sequence){
            
            std::string lidarPath, imagePath, timesPath, posesPath, calibPath;

            lidarPath = dataPath + "/data_odometry_velodyne/dataset/" + zeroPadding(sequence, 2) + "/velodyne";
            imagePath = dataPath + "/data_odometry_gray/dataset/sequences/" + zeroPadding(sequence, 2) + "/image_1"; // Need to change;
            timesPath = dataPath + "/data_odometry_calib/dataset/sequences/"  + zeroPadding(sequence, 2) + "/times.txt";;

            posesPath = dataPath + "/data_odometry_poses/dataset/poses/"  + zeroPadding(sequence, 2) + ".txt";
            calibPath = dataPath + "/data_odometry_calib/dataset/sequences/"  + zeroPadding(sequence, 2) + "/calib.txt";

            lidar_ = listdir(lidarPath, ".bin");
            image_ = listdir(imagePath, ".png");

            loadCalibData(calibPath);
            loadPosesData(posesPath);
            loadTimesData(timesPath);
        };

        KITTI_MAPPING(){

        };


        mapping();


    private:

        struct calib
        {
            cv::Mat Intrinsic;
            cv::Mat Extrinsic_R;
            cv::Mat Extrinsic_T;
            cv::Mat K;
        };
        
        std::vector<std::string> lidar_, image_;
        std::vector<double> times_;
        std::vector<cv::Mat> poses_;

        calib calib_;

        void loadTimesData(const std::string &timesPath);
        void loadPosesData(const std::string &posesPath);
        void loadCalibData(const std::string &calibPath);
};

#endif