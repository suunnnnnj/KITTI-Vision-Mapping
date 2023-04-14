#include "Mapping.hpp"

int main(int argc, char const *argv[])
{
    std::string dataPath = argv[1];
    std::string savePath = argv[2];
    int sequence = std::stoi(argv[3]);

    KITTI_MAPPING kitti(dataPath, savePath, sequence);
    kitti.Mapping();
}