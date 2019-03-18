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

//*************function declarations*****************//

pcl::visualization::PCLVisualizer::Ptr twoViewports(  //render two PointXYZRGB PointCloud together
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr,
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr);

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr);   //render a single PoinxXYZ PointCloud
pcl::visualization::PCLVisualizer::Ptr twoViewportsBW(  //render two PointXYZ Pointcloud Together
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);
pcl::visualization::PCLVisualizer::Ptr twoViewportsBlank(int & v1, int & v2);
void showCloudsRight(pcl::visualization::PCLVisualizer::Ptr viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, int & v2);
void showCloudsLeft(pcl::visualization::PCLVisualizer::Ptr viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, int & v1);
void showCloudLeftNormal(pcl::visualization::PCLVisualizer::Ptr viewer, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source, int & v);


#endif //PCL_REAL_VISUALIZER_H
