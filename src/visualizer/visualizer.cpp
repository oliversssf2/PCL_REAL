//
// Created by fongsu on 3/2/19.
//

#include "../includes/visualizer.h"

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
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

    int v2(1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0,0,0,v2);
    viewer->addText("Radius: 0.01", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGB> rgb2(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud2, rgb2, "sample cloud2", v2);

    return (viewer);
}

