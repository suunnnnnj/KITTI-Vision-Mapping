// tests/test_Mapper.cpp
#include <gtest/gtest.h>
#include "core/Mapper.hpp"
#include <filesystem>

using namespace kitti_mapping::core;

class MapperTest : public ::testing::Test {
protected:
    std::string imgDir = "test_imgs";
    std::string poseFile = "test_poses.txt";
    std::string outLAS = "out.las";

    void SetUp() override {
        std::filesystem::create_directory(imgDir);
        // create one dummy image
        cv::Mat dummy = cv::Mat::zeros(10, 10, CV_8UC3);
        cv::imwrite(imgDir + "/000000.png", dummy);
        // create pose file with identity
        std::ofstream ofs(poseFile);
        for (int i = 0; i < 12; ++i) {
            ofs << (i % 5 == 0 ? 1.0f : 0.0f) << (i < 11 ? " " : "");
        }
    }

    void TearDown() override {
        std::filesystem::remove_all(imgDir);
        std::filesystem::remove(poseFile);
        std::filesystem::remove(outLAS);
    }
};

TEST_F(MapperTest, RunGeneratesLAS) {
    Mapper mapper(imgDir, "%06d.png", poseFile, outLAS);
    mapper.run();
    ASSERT_TRUE(std::filesystem::exists(outLAS));
}

