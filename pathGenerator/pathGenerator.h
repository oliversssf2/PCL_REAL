//
// Created by fongsu on 3/21/19.
//

#ifndef PCL_REAL_PATHGENERATOR_H
#define PCL_REAL_PATHGENERATOR_H

#include <iostream>
#include <vector>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>


#include <librealsense2/rs.hpp>
#include "../realsense/cam_util.h"

struct PC_Data{ //PointCloud DATA
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> original_Clouds;
    std::vector<PCDPointNormal> processed_Clouds;
    std::vector<std::vector<size_t>> indices_order;

    int count = 0;
};

class pathGenerator {
public:
    pathGenerator();
    void Gen_compute();


protected:
    void PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output, pcl::PointCloud<pcl::PointXYZ>::Ptr searchSurface);
    void NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output);
    void NaNRemoval(pcl::PointCloud<pcl::PointNormal>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output);
    void Reorganize(pcl::PointCloud<pcl::PointNormal>::Ptr input);

    void setPassLimit(float up, float low){ passUpLim = up; passLowLim = low;};

private:
    rs2::pipeline pipe;
    rs2::points points;
    rs2::pointcloud pc;
    rs2::pipeline_profile pipe_profile;

    PC_Data data;

    //*downsample
    int downsample = 225;

    //**passthrough properties
    float passUpLim = 0.4;
    float passLowLim = 0.2;
    std::string passfield = "z";

    //**StatisticalOutlierRemoval properties
    int MeanK = 50;
    float StddevMulThresh = 1.0;

    //**NormalEstimation properties
    float searchRadius = 0.03;

    //**reorganize properties
    float reorganizeRange = 0.01;


};


#endif //PCL_REAL_PATHGENERATOR_H
