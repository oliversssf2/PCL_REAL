//
// Created by fongsu on 3/2/19.
//
#include <iostream>
#include <cstdio>

#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>

#include "../visualizer/visualizer.h"
#include "../realsense/cam_util.h"

#include <librealsense2/rs.hpp>

int main() {
    rs2::pipeline pipe;

    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline_profile p = pipe.start();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

    struct PCD{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::string f_name;
        PCD() :cloud (new pcl::PointCloud<pcl::PointXYZ>){};
    };

    std::vector<PCD> Clouds;

//******************************read pointcloud from camera******************************//
    char t;
    while (true)
    {
       std::cout << "press /'h/' to capture or press '/q'/ to quit" << std::endl;
       while(!scanf("%c", &t)){}
       if(t == 'h')
       {
           auto frames = pipe.wait_for_frames();
           auto depth = frames.get_depth_frame();
           points = pc.calculate(depth);
           std::string str = "cloud_" + std::to_string(Clouds.size()) + ".pcd";
           PCD m;
           m.f_name = str;
           m.cloud = points_to_pcl(points);
           std::vector<int> indices;
           pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
           Clouds.push_back(m);
       } else if(t == 'q')
           break;
    }
//******************************************************************************************//




    auto viewer = twoViewportsBW(clouds[0], clouds[1]);

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }


////********************i am a line*********************************////
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//
//    pcl::io::loadPCDFile<pcl::PointXYZ>("../prefabs/table_scene_lms400.pcd", *cloud);
//
//    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//              << " data points (" << pcl::getFieldsList(*cloud) << ")>";
//
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.01f, 0.01f, 0.01f);
//    sor.filter(*cloud_filtered);
//
//    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
//
//
//    auto viewer = twoViewportsBW(cloud, cloud_filtered);
//
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
////    ne.setRadiusSearch()
//
//
//    while(!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
//    }
////************************************i am another line*******************************************////
}

