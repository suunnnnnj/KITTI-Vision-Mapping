// tests/test_ImageLoader.cpp
#include <gtest/gtest.h>
#include "io/ImageLoader.hpp"
#include <filesystem>

using namespace kitti_mapping::io;

class ImageLoaderTest : public ::testing::Test {
protected:
    std::string tmpDir = "test_images";
    std::vector<std::string> filenames = {"000001.png", "000002.png"};

    void SetUp() override {
        std::filesystem::create_directory(tmpDir);
        for (auto& fn : filenames) {
            std::ofstream(tmpDir + "/" + fn).put('x');
        }
    }

    void TearDown() override {
        std::filesystem::remove_all(tmpDir);
    }
};

TEST_F(ImageLoaderTest, ScanAndOrder) {
    ImageLoader loader(tmpDir, "%06d.png");
    ASSERT_EQ(loader.size(), filenames.size());
}

TEST_F(ImageLoaderTest, NextAndReset) {
    ImageLoader loader(tmpDir, "%06d.png");
    auto img1 = loader.next();
    ASSERT_FALSE(img1.empty() || img1.data == nullptr);  // file exists but invalid image yields empty mat
    loader.reset();
    auto img2 = loader.next();
    ASSERT_FALSE(img2.empty() || img2.data == nullptr);
}

