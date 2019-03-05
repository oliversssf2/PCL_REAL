//
// Created by fongsu on 3/2/19.
//
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "../visualizer/visualizer.h"

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../prefabs/table_scene_lms400.pcd", *cloud);

    std::cerr << "PointCloude before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")>";

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";


    auto viewer = twoViewports(cloud, cloud_filtered);

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
}

