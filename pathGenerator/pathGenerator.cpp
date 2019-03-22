//
// Created by fongsu on 3/21/19.
//

#include <pcl/filters/statistical_outlier_removal.h>
#include "pathGenerator.h"

pathGenerator::pathGenerator() {
    pipe_profile = pipe.start();
}

void pathGenerator::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(passfield);
    pass.filter(*output);
}

void pathGenerator::StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_sort;
    stat_sort.setInputCloud(input);
    stat_sort.setMeanK(MeanK);
    stat_sort.setStddevMulThresh(StddevMulThresh);
    stat_sort.filter(*output);
}

void pathGenerator::NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output, pcl::PointCloud<pcl::PointXYZ>::Ptr searchSurface) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(input);
    ne.setSearchSurface(searchSurface);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(searchRadius);
    ne.compute(*output);

    pcl::copyPointCloud(*input, *output);
}

void pathGenerator::NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input,pcl::PointCloud<pcl::PointNormal>::Ptr output) {
    NormalEstimation(input, output, input);
}

void pathGenerator::NaNRemoval(pcl::PointCloud<pcl::PointNormal>::Ptr input,pcl::PointCloud<pcl::PointNormal>::Ptr output) {
    std::vector<int> indices_p; //NaN point indices
    std::vector<int> indices_n; //NaN normal indices

    pcl::PointCloud<pcl::PointNormal>::Ptr tem (new pcl::PointCloud<pcl::PointNormal>);
    pcl::removeNaNFromPointCloud(*input, *tem, indices_p);
    pcl::removeNaNNormalsFromPointCloud(*tem, *output, indices_n);
}

void pathGenerator::Reorganize(pcl::PointCloud<pcl::PointNormal>::Ptr input) {
    std::vector<size_t> idx(input->width);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&input](size_t i, size_t i2){ return (input->points.at(i).x < input->points.at(i2).x);});
    for(auto v : idx)
        std::cout << input->points.at(v) << std::endl;
    auto it_tgt = idx.begin();
    for(auto it_flag = idx.begin(); it_flag != idx.end(); it_flag++)
    {
        if((input->points.at(*it_flag).x - input->points.at(*it_tgt).x) > reorganizeRange)
        {
            std::sort(it_tgt, it_flag, [&input](size_t i, size_t i2){ return (input->points.at(i).y < input->points.at(i2).y);});
            it_tgt = it_flag;
        }
    }
    for(auto v : idx)
        std::cout << input->points.at(v) << std::endl;
    data.indices_order.push_back(idx);
}


void pathGenerator::Gen_compute() {
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);

    ++data.count;
    PCDPointNormal m;
    std::string str = "cloud_" + std::to_string(data.count-1); //give the Pointcloud a name
    m.f_name = str;
    m.index = data.count-1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr original = points_to_pcl(points);
    data.original_Clouds.push_back(original);

    pcl::PointCloud<pcl::PointXYZ>::Ptr stage_1 = points_to_pcl(points, downsample);
    pcl::PointCloud<pcl::PointXYZ>::Ptr stage_2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr stage_3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr stage_4 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr stage_5 (new pcl::PointCloud<pcl::PointNormal>);

    PassThrough(stage_1, stage_2);
    StatisticalOutlierRemoval(stage_2, stage_3);
    NormalEstimation(stage_3, stage_4);
    NaNRemoval(stage_4, stage_5);
    Reorganize(stage_5);

    data.processed_Clouds.push_back(m);
    savePointNormal(m, false, true, data.indices_order.back());
}

