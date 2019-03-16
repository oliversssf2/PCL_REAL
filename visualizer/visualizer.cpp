//
// Created by fongsu on 3/2/19.
//

#include "visualizer.h"

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return viewer;
}

pcl::visualization::PCLVisualizer::Ptr twoViewports(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0,0,0,v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud", v1);

    int v2(1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0,0,0,v2);
    viewer->addText("Radius: 0.01", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud2, rgb2, "sample cloud2", v2);

    viewer->initCameraParameters();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr twoViewportsBW(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0,0,0,v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    int v2(1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0,0,0,v1);
    viewer->addText("Radius: 0.01", 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud2", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr twoViewportsBlank(int & v1, int & v2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0,0,0,v1);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0,0,0,v1);

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return (viewer);
}

void showCloudsLeft(pcl::visualization::PCLVisualizer::Ptr viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, int & v1)
{
    viewer->removePointCloud("vp1_target");
    viewer->removePointCloud("vp1_source");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 255,0 , 0);

    viewer->addPointCloud(cloud_target, tgt_h, "vp1_target", v1);
    viewer->addPointCloud(cloud_source, src_h, "vp1_source", v1);

    viewer->spin();
}

void showCloudsRight(pcl::visualization::PCLVisualizer::Ptr viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, int & v2)
{
    viewer->removePointCloud ("source");
    viewer->removePointCloud ("target");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 255,0 , 0);

    viewer->addPointCloud(cloud_target, tgt_h, "target", v2);
    viewer->addPointCloud(cloud_source, src_h, "source", v2);

    viewer->spinOnce();
}

