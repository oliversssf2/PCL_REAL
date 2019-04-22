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
	rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
	window app(1280, 720, "Realsense Capture");
	ImGui_ImplGlfw_Init(app, false);

	rs2::colorizer color_map;
	rs2::rates_printer printer;
    int index = 0;
    pathGenerator gen;
    gen.updateSettings();

	rs2::frameset data = gen.wait_for_frames();

	//std::thread distanceDisplayThread(distanceDisplay, std::ref(data));

	char t;
	bool my_panel_activate = false;
	while (app) {
		data = gen.wait_for_frames().apply_filter(printer).apply_filter(color_map);
		app.show(data);

		ImGui_ImplGlfw_NewFrame(1);
		ImGui::Begin("Panel", &my_panel_activate);
		if(ImGui::Button("Capture", {50, 50}))
		{
			gen.Gen_compute();
		}
		ImGui::Text("Distance from the middle : %f", gen.midDist());
		ImGui::End();
		ImGui::Render();
    }


}