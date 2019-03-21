//
// Created by fongsu on 3/10/19.
//
#include "cam_util.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <cmakeconfig.h>
#include <pcl/io/pcd_io.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width()/sqrt(k);
    cloud->height = sp.height()/sqrt(k);
    cloud->is_dense = false;
    cloud->points.resize(points.size()/(k));
    auto ptr = points.get_vertices();

    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr += k;
    }

    return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    return points_to_pcl(points, 1);
}


void savePointNormal(PCDPointNormal cloud, bool pcd, bool csv, const std::vector<size_t> &idx)
{
    if(pcd)
    {
        std::string name = PCDOUTPUTPREFIX + cloud.f_name + ".pcd";
        pcl::io::savePCDFileASCII(name, *cloud.cloud);
    }
    if(csv)
    {
        std::string name = CSVOUTPUTPREFIX + cloud.f_name + ".csv";
        std::cout << name << std::endl;
        std::ofstream myfile;
        myfile.open(name, std::ios::out);
        if(!myfile)
        {
            std::cerr << "cant";
            exit(1);
        }

        for(auto x : idx)
        {
            myfile << cloud.cloud->points.at(x).x * 1000 << ','
                    << cloud.cloud->points.at(x).y * 1000 << ','
                    << cloud.cloud->points.at(x).z * 1000 << ','
                    << cloud.cloud->points.at(x).normal_x * (-1) << ','
                    << cloud.cloud->points.at(x).normal_y * (-1) << ','
                    << cloud.cloud->points.at(x).normal_z * (-1) << std::endl;
        }
        myfile.close();
    }
}

void savePointNormal(PCDPointNormal cloud, bool pcd, bool csv)
{
    std::vector<size_t> idx(cloud.cloud->size());
    std::iota(idx.begin(), idx.end(), 0);
    savePointNormal(cloud, pcd, csv, idx);
}

