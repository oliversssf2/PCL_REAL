//
// Created by fongsu on 3/10/19.
//

#ifndef PCL_REAL_CAM_UTIL_H
#define PCL_REAL_CAM_UTIL_H

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl-1.9/pcl/point_cloud.h>

struct PCDPointNormal {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
    std::string f_name;

    PCDPointNormal() : cloud(new pcl::PointCloud<pcl::PointNormal>) {};
};


pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points, int k=225);
void savePointNormal(PCDPointNormal cloud, bool pcd, bool csv, const std::vector<size_t> &idx);
void savePointNormal(PCDPointNormal cloud, bool pcd, bool csv);

#endif //PCL_REAL_CAM_UTIL_H
