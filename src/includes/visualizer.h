//
// Created by fongsu on 3/2/19.
//



#ifndef PCL_REAL_VISUALIZER_H
#define PCL_REAL_VISUALIZER_H

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr twoViewports(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr,
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr);

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr);

#endif //PCL_REAL_VISUALIZER_H
