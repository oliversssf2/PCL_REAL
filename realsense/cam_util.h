//
// Created by fongsu on 3/10/19.
//

#ifndef PCL_REAL_CAM_UTIL_H
#define PCL_REAL_CAM_UTIL_H

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl-1.9/pcl/point_cloud.h>

struct PCD{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::string f_name;

    PCD() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {};
};

struct PCDPointNormal {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
    std::string f_name;
    int index = -1;

    PCDPointNormal() : cloud(new pcl::PointCloud<pcl::PointNormal>) {};
};


pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points, int k);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points);
std::pair<std::string, std::string> savePointNormal(PCDPointNormal cloud, bool pcd, bool csv, const std::vector<size_t> &idx);
std::pair<std::string, std::string> savePointNormal(PCDPointNormal cloud, bool pcd, bool csv);

void readCoords(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);

#endif //PCL_REAL_CAM_UTIL_H
