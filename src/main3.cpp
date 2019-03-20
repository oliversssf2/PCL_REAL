//
// Created by fongsu on 3/18/19.
//
#include <iostream>
#include <fstream>
#include <vector>

#include <librealsense2/rs.hpp>
#include "../realsense/cam_util.h"
#include "../visualizer/visualizer.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


int main()
{
    rs2::pipeline pipe;

    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline_profile p = pipe.start();

    std::vector<PCDPointNormal> Clouds;

    char t;
    int index = 0;
    while (true) {
        std::cout << "press /'h/' to capture or press '/q'/ to quit" << std::endl;
        while (!scanf("%c", &t)) {}
        if (t == 'h') {
            int v1, v2;
            auto viewer = twoViewportsBlank(v1, v2);

            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            points = pc.calculate(depth);

            std::string str = "cloud_" + std::to_string(Clouds.size()) + ".pcd";
            PCDPointNormal m;
            m.f_name = str;

            pcl::PointCloud<pcl::PointXYZ>::Ptr i = points_to_pcl(points);
            pcl::PointCloud<pcl::PointXYZ>::Ptr i_2 (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(i);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.2, 0.4);
            pass.filter(*i_2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr i_3 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_sort;
            stat_sort.setInputCloud(i_2);
            stat_sort.setMeanK(50);
            stat_sort.setStddevMulThresh(3.0);
            stat_sort.filter(*i_3);

            pcl::PointCloud<pcl::PointNormal>::Ptr i_4 (new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr i_5 (new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr blank (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointNormal>::Ptr blankNormal (new pcl::PointCloud<pcl::PointNormal>);

            showCloudsLeft(viewer, i_2, blank, v1);

            pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
            ne.setInputCloud(i_3);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(0.03);

            ne.compute(*i_4);
            pcl::copyPointCloud(*i_2, *i_4);

            std::cout << "Show Normal Cloud Now" << endl;
            showCloudLeftNormal(viewer, i_4, blankNormal, v2);

            std::vector<int> indices1;
            pcl::removeNaNFromPointCloud(*i_4, *i_5, indices1);
            std::vector<int> indices2;
            pcl::removeNaNNormalsFromPointCloud(*i_5, *m.cloud, indices2);

            m.f_name = "cloud_" + std::to_string(index);

            std::vector<size_t> idx(m.cloud->width);
            std::iota(idx.begin(), idx.end(), 0);
            std::sort(idx.begin(), idx.end(), [&m](size_t i, size_t i2){ return (m.cloud->points.at(i).x < m.cloud->points.at(i2).x);});
            for(auto v : idx)
                std::cout << m.cloud->points.at(v) << std::endl;
            for(size_t j = 0; j < idx.size(); j++)
            {
                ;
            }

            Clouds.push_back(m);
            savePointNormal(m, false, true);
            ++index;
        } else if (t == 'q')
            break;
    }

}