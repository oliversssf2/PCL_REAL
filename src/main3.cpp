//
// Created by fongsu on 3/18/19.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <imgui.h>
#include <imgui_impl_glfw.h>

#include <librealsense2/rs.hpp>
#include <example.hpp>
#include "../realsense/cam_util.h"
#include "../visualizer/visualizer.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "../pathGenerator/pathGenerator.h"

void distanceDisplay(rs2::frameset &data) {
	while (true) {
		rs2::depth_frame depth = data.get_depth_frame();
		float dist_to_center = depth.get_distance(depth.get_width() / 2, depth.get_height() / 2);
	}
}

int main()
{
	//rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
	window app(1280, 480, "Realsense Capture");
	ImGui_ImplGlfw_Init(app, false);

	rs2::colorizer color_map;
	rs2::rates_printer printer;
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
    int index = 0;
    pathGenerator gen;
    gen.updateSettings();

	rs2::frameset data = gen.wait_for_frames();

	float clipDist = 0.0f;
	float voxelLeafSize = 0.0f;

	//std::thread distanceDisplayThread(distanceDisplay, std::ref(data));

	char t;
	bool my_panel_activate = false;
	while (app) {
		data = gen.wait_for_frames();
		data = align_to_depth.process(data);
		data = data.apply_filter(printer).apply_filter(color_map);
		app.show(data);

		ImGui_ImplGlfw_NewFrame(1);
		ImGui::Begin("Panel", &my_panel_activate);
		if (ImGui::Button("Capture", {50, 50})) {
			gen.Gen_compute();
		}

		ImGui::SliderFloat("MaxClipDist", &clipDist, 0.16, 1.1);
		ImGui::SliderFloat("Voxel Grid Size", &voxelLeafSize, 0, 0.1);

		ImGui::Text("Distance from the middle : %f", gen.midDist());
		ImGui::End();
		ImGui::Render();
		//Render the UI
    }
	//if(distanceDisplayThread.joinable())
	//	distanceDisplayThread.join();
//    while (true) {
//        std::cout << "press /'h/' to capture or press '/q'/ to quit" << std::endl;
//        while (!scanf("%c", &t)) {}
//        if (t == 'h') {
////            int v1, v2;
////            auto viewer = twoViewportsBlank(v1, v2);
////
////            auto frames = pipe.wait_for_frames();
////            auto depth = frames.get_depth_frame();
////            points = pc.calculate(depth);
////
////            std::string str = "cloud_" + std::to_string(Clouds.size()) + ".pcd";
////            PCDPointNormal m;
////            m.f_name = str;
////
////            pcl::PointCloud<pcl::PointXYZ>::Ptr original = points_to_pcl(points);
////            pcl::PointCloud<pcl::PointXYZ>::Ptr i = points_to_pcl(points, 400);
////            pcl::PointCloud<pcl::PointXYZ>::Ptr i_2 (new pcl::PointCloud<pcl::PointXYZ>);
////
////            pcl::PassThrough<pcl::PointXYZ> pass;
////            pass.setInputCloud(i);
////            pass.setFilterFieldName("z");
////            pass.setFilterLimits(0.2, 0.4);
////            pass.filter(*i_2);
////
////            pcl::PointCloud<pcl::PointXYZ>::Ptr i_3 (new pcl::PointCloud<pcl::PointXYZ>);
////            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_sort;
////            stat_sort.setInputCloud(i_2);
////            stat_sort.setMeanK(50);
////            stat_sort.setStddevMulThresh(1.0);
////            stat_sort.filter(*i_3);
////
////            pcl::PointCloud<pcl::PointNormal>::Ptr i_4 (new pcl::PointCloud<pcl::PointNormal>);
////            pcl::PointCloud<pcl::PointNormal>::Ptr i_5 (new pcl::PointCloud<pcl::PointNormal>);
////            pcl::PointCloud<pcl::PointXYZ>::Ptr blank (new pcl::PointCloud<pcl::PointXYZ>);
////            pcl::PointCloud<pcl::PointNormal>::Ptr blankNormal (new pcl::PointCloud<pcl::PointNormal>);
////
////            showCloudsLeft(viewer, i_2, blank, v1);
////
////            pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
////            ne.setInputCloud(i_3);
////            ne.setSearchSurface(original);
////            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
////            ne.setSearchMethod(tree);
////            ne.setRadiusSearch(0.03);
////
////            ne.compute(*i_4);
////            pcl::copyPointCloud(*i_2, *i_4);
////
////            std::cout << "Show Normal Cloud Now" << endl;
////            showCloudLeftNormal(viewer, i_4, blankNormal, v2);
////
////            std::vector<int> indices1;
////            pcl::removeNaNFromPointCloud(*i_4, *i_5, indices1);
////            std::vector<int> indices2;
////            pcl::removeNaNNormalsFromPointCloud(*i_5, *m.cloud, indices2);
////
////            m.f_name = "cloud_" + std::to_string(index);
////
////            std::vector<size_t> idx(m.cloud->width);
////            std::iota(idx.begin(), idx.end(), 0);
////            std::sort(idx.begin(), idx.end(), [&m](size_t i, size_t i2){ return (m.cloud->points.at(i).x < m.cloud->points.at(i2).x);});
////            for(auto v : idx)
////                std::cout << m.cloud->points.at(v) << std::endl;
////            auto it_tgt = idx.begin();
////            for(auto it_flag = idx.begin(); it_flag != idx.end(); it_flag++)
////            {
////                if((m.cloud->points.at(*it_flag).x - m.cloud->points.at(*it_tgt).x) > 0.01)
////                {
////                    std::sort(it_tgt, it_flag, [&m](size_t i, size_t i2){ return (m.cloud->points.at(i).y < m.cloud->points.at(i2).y);});
////                    it_tgt = it_flag;
////                }
////            }
////            for(auto v : idx)
////                std::cout << m.cloud->points.at(v) << std::endl;
////
////
////            Clouds.push_back(m);
////            savePointNormal(m, false, true, idx);
////            ++index;
//            gen.Gen_compute();
//
//
//        } else if (t == 'q')
//            break;
//    }

}