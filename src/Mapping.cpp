#include "Mapping.hpp"

std::vector<std::string> KITTI_MAPPING::loadTimesData(const std::string &timesPath)
{

}

std::vector<std::string> KITTI_MAPPING::loadPosesData(const std::string &calibPath)
{

}


std::vector<std::string> KITTI_MAPPING::loadCalibData(const std::string &calibPath)
{   
    std::ifstream ifs(calibPath);

    if(ifs.is_open())
    {
        std::string line;
        std::getline(ifs, line);
        std::getline(ifs, line); // image_1;
        

    }

}