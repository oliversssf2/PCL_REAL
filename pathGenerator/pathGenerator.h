//
// Created by fongsu on 3/21/19.
//

#ifndef PCL_REAL_PATHGENERATOR_H
#define PCL_REAL_PATHGENERATOR_H

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <fstream>

#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>


#include <librealsense2/rs.hpp>
#include "../realsense/cam_util.h"

#include <cmakeconfig.h>


struct PC_Data{ //PointCloud DATA
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> original_Clouds;
    std::vector<PCDPointNormal> processed_Clouds;
    std::vector<std::vector<size_t>> indices_order;

    int count = 0;
};

class pathGenerator {
public:
    pathGenerator();
    ~pathGenerator();
    void Gen_compute();
    void updateSettings();

	rs2::frameset wait_for_frames();
	float midDist();

protected:
    void PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output, pcl::PointCloud<pcl::PointXYZ>::Ptr searchSurface);
    void NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output);
    void NaNRemoval(pcl::PointCloud<pcl::PointNormal>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output);
    void Reorganize(pcl::PointCloud<pcl::PointNormal>::Ptr input);

    inline void setPassLimit(float _limitMax, float _limitMin, std::string _passfield){ limitMax = _limitMax; limitMin = _limitMin; passfield = _passfield;}
    inline void setDownsample(int _downsample){downsample = _downsample;}
    inline void setStatOutRem(int _MeanK, float _StddevMulThresh){MeanK = _MeanK; StddevMulThresh = _StddevMulThresh;}
    inline void setNormalEst(float _searchRadius){searchRadius = _searchRadius;}

private:
    rs2::pipeline pipe;
    rs2::points points;
    rs2::pointcloud pc;
    rs2::pipeline_profile pipe_profile;

    PC_Data data;
    rs2::frameset frames;

    //*downsample
    int downsample = 400;

    //**passthrough properties
    float limitMax = 0.35f;
	float limitMin = 0.15f;
    std::string passfield = "z";

    //**StatisticalOutlierRemoval properties
    int MeanK = 30;
    float StddevMulThresh = 1.0;

    //**NormalEstimation properties
    float searchRadius = 0.03;

    //**reorganize propertiesh

    float reorganizeRange = 0.008;

    std::string distFileName;
	std::ofstream distfile;
};


#endif //PCL_REAL_PATHGENERATOR_H
