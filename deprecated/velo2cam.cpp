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

    auto Lidar2Cam::splitString(const std::string& str, const char delim) -> std::vector<std::string> {
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
            std::cerr << "Failed to open calibration file: " << file_path << std::endl;
            return;
        }

        std::string line;
        while(std::getline(calib_file, line)) {
            std::vector<std::string> tokens = splitString(line, ' ');

            if(tokens[0] == "P0:") {
                p0_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                       std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                       std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]);
            } else if (tokens[0] == "P1:") {
                p1_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                       std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                       std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]);
            } else if (tokens[0] == "P2:") {
                p2_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                       std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                       std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]);
            } else if (tokens[0] == "P3:") {
                p3_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                       std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                       std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]);
            } else if (tokens[0] == "R0_rect:") {
                r0_rect_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]),
                            std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]),
                            std::stod(tokens[7]), std::stod(tokens[8]), std::stod(tokens[9]);
            } else if (tokens[0] == "Tr_velo_to_cam:") {
                tr_velo_to_cam_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                                   std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                                   std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]);
            } else if (tokens[0] == "Tr_imu_to_velo:") {
                tr_imu_to_velo_ << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                                   std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                                   std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]);
            }
        }

        calib_file.close();
    }

    void Lidar2Cam::projectionLidar2Cam() {
        std::cout << "projectionLidar2Cam" << std::endl;

        cloud_cam_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        
        for(const auto& point : cloud_->points) {
            Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1.0);            
            Eigen::Vector3d point_cam = p2_ * (r0_rect_ * tr_velo_to_cam_ * point_lidar).homogeneous();
            point_cam /= point_cam[2];

            cloud_cam_->points.push_back(pcl::PointXYZ(point_cam[0], point_cam[1], point_cam[2]));
        }
    }

    void Lidar2Cam::printPointCloud() {
        for(const auto& point : cloud_->points) {
            std::cout << "[" << cloud_->size() << "] : " << point.x << " " << point.y << " " << point.z << std::endl;
        }
    }

    void Lidar2Cam::printCalib() {
        std::cout << "P0: " << std::endl << p0_ << std::endl;
        std::cout << "P1: " << std::endl << p1_ << std::endl;
        std::cout << "P2: " << std::endl << p2_ << std::endl;
        std::cout << "P3: " << std::endl << p3_ << std::endl;
        std::cout << "R0_rect: " << std::endl << r0_rect_ << std::endl;
        std::cout << "Tr_velo_to_cam: " << std::endl << tr_velo_to_cam_ << std::endl;
        std::cout << "Tr_imu_to_velo: " << std::endl << tr_imu_to_velo_ << std::endl;
    }
}