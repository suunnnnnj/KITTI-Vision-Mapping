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

            Eigen3x4d pose;
            pose << boost::lexical_cast<double>(tokens[0]), boost::lexical_cast<double>(tokens[1]), boost::lexical_cast<double>(tokens[2]), boost::lexical_cast<double>(tokens[3]),
                    boost::lexical_cast<double>(tokens[4]), boost::lexical_cast<double>(tokens[5]), boost::lexical_cast<double>(tokens[6]), boost::lexical_cast<double>(tokens[7]),
                    boost::lexical_cast<double>(tokens[8]), boost::lexical_cast<double>(tokens[9]), boost::lexical_cast<double>(tokens[10]), boost::lexical_cast<double>(tokens[11]);
            
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
        while(std::getline(ifs, line))
        {
            calib c;
            std::vector<std::string> tokens;

            tokens = tokenize(line);

            Eigen3x4d K;
            
            K << boost::lexical_cast<double>(tokens[1]), boost::lexical_cast<double>(tokens[2]), boost::lexical_cast<double>(tokens[3]), boost::lexical_cast<double>(tokens[4]),
                boost::lexical_cast<double>(tokens[5]), boost::lexical_cast<double>(tokens[6]), boost::lexical_cast<double>(tokens[7]), boost::lexical_cast<double>(tokens[8]),
                boost::lexical_cast<double>(tokens[9]), boost::lexical_cast<double>(tokens[10]), boost::lexical_cast<double>(tokens[11]), boost::lexical_cast<double>(tokens[12]);

            c.K = K;

            calib_.push_back(c);
        }
    }

    ifs.close();

    return;
}

void KITTI_MAPPING::Mapping()
{
    tqdm bar;
    pcl::PointCloud<pcl::PointXYZRGBI>::Ptr map(new pcl::PointCloud<pcl::PointXYZRGBI>());

    int mIdx = 0;
    for(int fIdx = 0; fIdx < nof_; fIdx++)
    {
        bar.progress(fIdx, nof_);

        std::ifstream lidar(lidar_[fIdx], std::ios::binary);

        lidar.seekg(0, std::ios::end);
        size_t nop = lidar.tellg() / sizeof(pt); // number of points;
        lidar.seekg(0, std::ios::beg);

        for(int ptIdx = 0 ; ptIdx < nop; ptIdx++)
        {
            pt pt_;
            lidar.read((char*)&pt_, sizeof(pt_));

            Eigen4x1d lidar_h;
            lidar_h << pt_.x, pt_.y, pt_.z, 1; 

            Eigen3x1d camera = calib_[4].K * lidar_h;
            
            Eigen4x1d camera_h; 
            camera_h << camera(0), camera(1), camera(2), 1;

            Eigen3x1d world = poses_[fIdx] * camera_h;

            pcl::PointXYZRGBI pcl_pt;

            pcl_pt.x = world(0);
            pcl_pt.y = world(1);
            pcl_pt.z = world(2);
            pcl_pt.intensity = pt_.intensity;
            pcl_pt.r = 0;
            pcl_pt.g = 0;
            pcl_pt.b = 0;

            map->points.push_back(pcl_pt);
        }

        if(fIdx != 0 && fIdx % 100 == 0)
        {
            std::string mapPath = savePath_ + zeroPadding(mIdx, 2) + ".pcd";
            pcl::io::savePCDFileBinary(mapPath, *map);
            map->points.clear();
            mIdx++;
        }
    }

    if( map->points.size() > 0 )
    {
        std::string mapPath = savePath_ + zeroPadding(mIdx, 2) + ".pcd";
        pcl::io::savePCDFileBinary(mapPath, *map);
    }

    return;
}

void KITTI_MAPPING::Visualization()
{
    return;
}