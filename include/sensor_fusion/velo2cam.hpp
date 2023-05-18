#pragma once

#include "rclcpp/rclcpp.hpp"

#include "Eigen/Dense"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <string>

namespace sensor_fusion {
    class Lidar2Cam {
    public:
        Lidar2Cam() = default;
        ~Lidar2Cam() = default;

        void loadPointCloud(const std::string& file_path);
        void loadCalib(const std::string& file_path);

        void projectionLidar2Cam();

        void printPointCloud();
        void printCalib();
    private:
        std::vector<std::string> splitString(const std::string& str, const char delim);
    private:
        // Eigen::Matrix2d camera_;

        // point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam_;

        // calibration parameters
        Eigen::Matrix<double, 3, 4> p0_;
        Eigen::Matrix<double, 3, 4> p1_;
        Eigen::Matrix<double, 3, 4> p2_; // use this for sample_data/images
        Eigen::Matrix<double, 3, 4> p3_;
        Eigen::Matrix<double, 3, 3> r0_rect_;
        Eigen::Matrix<double, 3, 4> tr_velo_to_cam_;
        Eigen::Matrix<double, 3, 4> tr_imu_to_velo_;

    };
} // namespace sensor_fusion