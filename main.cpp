#include "Mapping.hpp"

int main(int argc, char const *argv[])
{
    std::string dataPath = argv[1];
    int sequence = std::stoi(argv[2]);

    KITTI_MAPPING kitti(dataPath, sequence);
}