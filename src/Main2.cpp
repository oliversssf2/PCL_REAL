//
// Created by fongsu on 3/17/19.
//

#include <iostream>
#include <fstream>

#include <librealsense2/rs.hpp>
#include "../realsense/cam_util.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <cmakeconfig.h>
#include "../visualizer/visualizer.h"


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

            auto i = points_to_pcl(points);

            pcl::PointCloud<pcl::PointXYZ>::Ptr i_2 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr i_3 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr i_4 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr blank (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (i);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 1);
            pass.filter (*i_2);

            showCloudsLeft(viewer, i, blank, v1);
            showCloudsRight(viewer, i_2, blank, v2);

            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud (i_2);
            vox.setLeafSize (0.005f, 0.005f, 0.005f);
            vox.filter (*i_3);

            showCloudsLeft(viewer, i_3, blank, v1);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat;
            stat.setInputCloud (i_3);
            stat.setMeanK (50);
            stat.setStddevMulThresh (2.0);
            stat.filter (*i_4);

            showCloudsRight(viewer, i_4, blank, v2);

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
            mls.setComputeNormals(true);
            mls.setInputCloud(i_4);
            mls.setPolynomialOrder(2);
            mls.setSearchMethod(tree);
            mls.setSearchRadius(0.03);
            mls.process(*m.cloud);

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

            std::cout << m.cloud->width << endl;

            m.f_name = "cloud_" + std::to_string(index);
            Clouds.push_back(m);
            savePointNormal(m, false, true);
            ++index;
        } else if (t == 'q')
            break;
    }

}


