//
// Created by fongsu on 3/10/19.
//

#ifndef PCL_REAL_CAM_UTIL_H
#define PCL_REAL_CAM_UTIL_H

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl-1.9/pcl/point_cloud.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points);

#endif //PCL_REAL_CAM_UTIL_H
