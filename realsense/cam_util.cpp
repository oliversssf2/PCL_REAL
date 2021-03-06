//
// Created by fongsu on 3/10/19.
//
#include "cam_util.h"
#include <fstream>
#include <cmakeconfig.h>
#include <pcl/io/pcd_io.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
//    cloud->width = sp.width()/sqrt(k);
//    cloud->height = sp.height()/sqrt(k);
    cloud->width = sp.width()/sqrt(k);
    cloud->height = sp.height()/sqrt(k);
    cloud->is_dense = false;
    cloud->points.resize(points.size()/(k));
    auto ptr = points.get_vertices();

    int width = cloud->width;
    int height = cloud->height;
    auto it = cloud->points.begin();


	for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            it->x = ptr->x;
            it->y = ptr->y;
            it->z = ptr->z;
            it++;
            ptr += int(sqrt(k));
        }
        ptr += int(sqrt(k) * sp.width());
    }

//    for (auto& p : cloud->points)
//    {
//        p.x = ptr->x;
//        p.y = ptr->y;
//        p.z = ptr->z;
//        ptr += k;
//    }

    return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    return points_to_pcl(points, 1);
}


std::pair<std::string, std::string> savePointNormal(PCDPointNormal cloud, bool pcd, bool csv, const std::vector<size_t> &idx)
{
    std::string csv_path, pcd_path;
    if(pcd)
    {
        std::string name = PCDOUTPUTPREFIX + cloud.f_name + ".pcd";
        pcl::io::savePCDFileASCII(name, *cloud.cloud);
        pcd_path = name;
    }
    if(csv)
    {
        std::string name = CSVOUTPUTPREFIX + cloud.f_name + ".csv";
        csv_path = name;
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
    return std::make_pair(csv_path, pcd_path);
}

std::pair<std::string, std::string> savePointNormal(PCDPointNormal cloud, bool pcd, bool csv)
{
    std::vector<size_t> idx(cloud.cloud->size());
    std::iota(idx.begin(), idx.end(), 0);
    return savePointNormal(cloud, pcd, csv, idx);
}

void readCoords(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file)
{
	std::ifstream myfile;
	myfile.open(file, std::ios::in);
	if(!myfile.is_open())
	{
		std::cerr << "CAN'T READ FILE" << std::endl;
		return;
	}
	size_t cnt = 0;
	for(auto &pt : cloud->points)
	{
		if(myfile >> pt.x >> pt.y)
		{
			pt.z = 0;
			std::cout << pt.x << '\t' << pt.y << std::endl;
			cnt++;
		}
		else
			break;
	}
	cloud->width = cnt;
	cloud->resize(cloud->width * cloud->height);
	myfile.close();
}

