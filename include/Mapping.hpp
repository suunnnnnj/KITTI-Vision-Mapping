#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "utils.hpp"
#include "tqdm.h"

class KITTI_MAPPING
{
    public:

        KITTI_MAPPING(const std::string &dataPath, const std::string &savePath, const int &sequence){
            
            savePath_ = savePath + "/" + zeroPadding(sequence, 2) + "/";
            
            if(!boost::filesystem::exists(savePath_))
                boost::filesystem::create_directories(savePath_);

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

            if(!all_equal(lidar_.size(), image_.size(), poses_.size(), times_.size()))
            {
                std::cout << "Data leak." << std::endl;
                std::cout << "You need to check Dataset..." << std::endl;
                std::cout << "lidar.size() = " << lidar_.size() << std::endl;
                std::cout << "image.size() = " << image_.size() << std::endl;
                std::cout << "poses.size() = " << poses_.size() << std::endl;
                std::cout << "times.size() = " << times_.size() << std::endl;
                exit(-1);
            }

            nof_ = lidar_.size();
        };

        KITTI_MAPPING(){

        };

        void Mapping();
        void Visualization();

    private:

        struct calib
        {
            Eigen3x4d K;
        };

        struct pt
        {
            float x, y, z, intensity;
        };

        std::string savePath_;

        std::vector<std::string> lidar_, image_;
        std::vector<double> times_;
        std::vector<Eigen3x4d> poses_;

        std::vector<calib> calib_;

        void loadTimesData(const std::string &timesPath);
        void loadPosesData(const std::string &posesPath);
        void loadCalibData(const std::string &calibPath);

        double nof_; // number of frame;
};

#endif