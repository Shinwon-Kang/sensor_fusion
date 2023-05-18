#include "velo2cam.hpp"

#include "gtest/gtest.h"

class TestClass : public testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(TestClass, test1) {
    sensor_fusion::Lidar2Cam lidar_2_cam;
    lidar_2_cam.loadPointCloud("/root/ros2_ws/src/sensor_fusion/sample_data/points/000031.pcd");
    lidar_2_cam.loadCalib("/root/ros2_ws/src/sensor_fusion/sample_data/calibs/000031.txt");

    lidar_2_cam.projectionLidar2Cam();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();

    return result;
}