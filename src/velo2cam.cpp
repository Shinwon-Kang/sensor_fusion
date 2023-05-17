#include "velo2cam.hpp"

#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"

#include <fstream>
#include <sstream>
#include <vector>

namespace sensor_fusion {
    void Lidar2Cam::loadPointCloud(const std::string& file_path) {
        cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(file_path, *cloud_);
    }

    auto Lidar2Cam::splitString(const std::string& str, const std::string& delim) -> std::vector<std::string> {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream token_stream(str);
        while(std::getline(token_stream, token, delim)) {
            tokens.push_back(token);
        }

        return tokens;
    }

    void Lidar2Cam::loadCalib(const std::string& file_path) {
        std::ifstream calib_file(file_path);
        if(!calib_file.is_open()) {
            // RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: {}", file_path);
            std::cerr << "Failed to open calibration file: " << file_path << std::endl;
            return;
        }

        std::string line;
        while(std::getline(calib_file, line)) {
            std::vector<std::string> tokens = splitString(line, " ");
            for(const auto& token : tokens) {
                // RCLCPP_INFO(this->get_logger(), "{}", token);
                std::cout << token << std::endl;
            }
            
        }
    }

    void Lidar2Cam::printPointCloud() {
        for(const auto& point : cloud_->points) {
            // RCLCPP_INFO(this->get_logger(), "[{}] : {} {} {}", cloud_->size(), point.x, point.y, point.z
            std::cout << "[" << cloud_->size() << "] : " << point.x << " " << point.y << " " << point.z << std::endl;
        }
    }
}