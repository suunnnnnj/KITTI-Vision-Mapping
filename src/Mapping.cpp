#include "Mapping.hpp"

void KITTI_MAPPING::loadTimesData(const std::string &timesPath)
{
    std::ifstream ifs(timesPath);

    if(ifs.is_open())
    {
        std::string line;
        
        while(std::getline(ifs, line))
        {
            double t = std::stod(line);
            times_.push_back(t);
        }
    }

    ifs.close();

    return;
}

void KITTI_MAPPING::loadPosesData(const std::string &posesPath)
{
    std::ifstream ifs(posesPath);

    if(ifs.is_open())
    {
        std::string line;
        
        while(std::getline(ifs, line))
        {
            std::vector<std::string> tokens;

            tokens = tokenize(line);

            cv::Mat pose = (cv::Mat_<double>(3, 4) << boost::lexical_cast<double>(tokens[0]), boost::lexical_cast<double>(tokens[1]), boost::lexical_cast<double>(tokens[2]), boost::lexical_cast<double>(tokens[3]),
                                                      boost::lexical_cast<double>(tokens[4]), boost::lexical_cast<double>(tokens[5]), boost::lexical_cast<double>(tokens[6]), boost::lexical_cast<double>(tokens[7]),
                                                      boost::lexical_cast<double>(tokens[8]), boost::lexical_cast<double>(tokens[9]), boost::lexical_cast<double>(tokens[10]), boost::lexical_cast<double>(tokens[11]));
            poses_.push_back(pose);
        }
    }

    ifs.close();
    
    return;
}


void KITTI_MAPPING::loadCalibData(const std::string &calibPath)
{   

    std::ifstream ifs(calibPath);

    if(ifs.is_open())
    {
        std::string line;
        std::getline(ifs, line);
        std::getline(ifs, line); // image_1;
        
        std::vector<std::string> tokens;

        tokens = tokenize(line);

        cv::Mat K = (cv::Mat_<double>(3, 4) << boost::lexical_cast<double>(tokens[1]), boost::lexical_cast<double>(tokens[2]), boost::lexical_cast<double>(tokens[3]), boost::lexical_cast<double>(tokens[4]),
                                               boost::lexical_cast<double>(tokens[5]), boost::lexical_cast<double>(tokens[6]), boost::lexical_cast<double>(tokens[7]), boost::lexical_cast<double>(tokens[8]),
                                               boost::lexical_cast<double>(tokens[9]), boost::lexical_cast<double>(tokens[10]), boost::lexical_cast<double>(tokens[11]), boost::lexical_cast<double>(tokens[12]));
        calib_.K = K;
    }

    ifs.close();

    return;
}