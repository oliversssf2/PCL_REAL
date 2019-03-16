//
// Created by fongsu on 3/2/19.
//
#include <iostream>

#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>

#include "../visualizer/visualizer.h"
#include "../realsense/cam_util.h"

int main() {
    rs2::pipeline pipe;

    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline_profile p = pipe.start();

    struct PCD {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::string f_name;

        PCD() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {};
    };

    std::vector<PCD> Clouds;


//******************************read pointcloud from camera******************************//
    char t;
    while (true) {
        std::cout << "press /'h/' to capture or press '/q'/ to quit" << std::endl;
        while (!scanf("%c", &t)) {}
        if (t == 'h') {
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            points = pc.calculate(depth);

            std::string str = "cloud_" + std::to_string(Clouds.size()) + ".pcd";
            PCD m;
            m.f_name = str;

            auto i = points_to_pcl(points);

            pcl::PointCloud<pcl::PointXYZ>::Ptr i_2 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr i_3 (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (i);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 1);
            pass.filter (*i_2);

            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud (i_2);
            vox.setLeafSize (0.005f, 0.005f, 0.005f);
            vox.filter (*i_3);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat;
            stat.setInputCloud (i_3);
            stat.setMeanK (50);
            stat.setStddevMulThresh (1.0);
            stat.filter (*m.cloud);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
            Clouds.push_back(m);
        } else if (t == 'q')
            break;
    }

    int v1, v2;
    auto viewer = twoViewportsBlank(v1, v2);

//******************************************************************************************//
    for (size_t i = 1; i < Clouds.size(); i++) {
        auto target = Clouds[i].cloud;
        auto source = Clouds[i - 1].cloud;

        showCloudsLeft(viewer, target, source, v1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_t(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_t(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::copyPointCloud(*source, *source_t);
        pcl::copyPointCloud(*target, *target_t);

        icp.setInputCloud(source_t);
        icp.setInputTarget(target_t);

        icp.setMaxCorrespondenceDistance(0.1);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1);
        icp.setMaximumIterations(10);

        pcl::PointCloud<pcl::PointXYZ>::Ptr result = source_t;


        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), targetToSource;
        for (size_t j = 1; j < 50; j++) {
            source_t = result;
            icp.setInputCloud(source_t);
            icp.align(*result);
            std::cout << "hi" << endl;
            transformation = icp.getFinalTransformation() * transformation;
            showCloudsRight(viewer, target_t, source_t, v2);
        }
        targetToSource = transformation.inverse();
        pcl::transformPointCloud(*target, *output, targetToSource);
        *output += *source;
        viewer->spin();
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

