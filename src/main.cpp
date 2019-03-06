//
// Created by fongsu on 3/2/19.
//
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "../visualizer/visualizer.h"

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stage_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stage_2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("../prefabs/table_scene_lms400.pcd", *cloud);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")>";

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_stage_1);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud(cloud_stage_1);
    sor2.setMeanK(50);
    sor2.setStddevMulThresh(1.0);
    sor2.filter(*cloud_stage_2);


    std::cerr << "PointCloud after filtering: " << cloud_stage_1->width * cloud_stage_1->height
              << " data points (" << pcl::getFieldsList(*cloud_stage_1) << ").";
    std::cerr << "PointCloud after filtering: " << cloud_stage_2->width * cloud_stage_2->height
              << " data points (" << pcl::getFieldsList(*cloud_stage_2) << ").";

    auto viewer = twoViewportsBW(cloud_stage_1, cloud_stage_2);

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(10000));
        std::cout << "HAHAHAHA" << endl;
    }
}

