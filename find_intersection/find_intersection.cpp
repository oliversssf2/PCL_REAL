//
// Created by fongsu on 4/21/19.
//
#include <iostream>
#include <thread>

#include <cam_util.h>
#include <cmakeconfig.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/intersections.h>

#include <intersection_finder.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters ();
	return (viewer);

}

int
main(int argc, char** argv)
{
	std::cout << "PRESS ENTER TO START!!!" << std::endl;
	while (std::cin.get()) {
		float threshold;
		std::string file1, file2;
		std::cout << "first file: ";
		std::cin >> file1;
		std::cout << std::endl;
		std::cout << "second file: ";
		std::cin >> file2;
		std::cout << std::endl;
		std::cout << "enter threshold" << std::endl;
		std::cin >> threshold;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::ModelCoefficients k;
		fitline(file1, final, k, threshold);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final_2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ModelCoefficients k_2;
		fitline(file2, final_2, k_2, threshold);

		Eigen::Vector4f intersect_pt;
		pcl::PointCloud<pcl::PointXYZ>::Ptr intersect(new pcl::PointCloud<pcl::PointXYZ>);
		find_intersection(k, k_2, intersect_pt, intersect);

		std::cout << "result: " << std::endl;
		for (auto &k : intersect->points)
			cout << "X: " << k.x << " Y:" << k.y << " Z:" << k.z << std::endl;
		std::cout << intersect_pt << std::endl;
		std::cout << "Press Enter to continue" << std::endl;

		pcl::visualization::PCLVisualizer::Ptr viewer;

		viewer = simpleVis(final);
		viewer->addPointCloud(final_2, "cloud_2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud_2");

		viewer->addPointCloud(intersect, "intersect");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "intersect");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "intersect");

		viewer->addCoordinateSystem(1);

		viewer->addLine(k, "line one");
		viewer->addLine(k_2, "line two");
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			std::this_thread::sleep_for(100ms);
		}
		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->close();
		viewer->close();
	}
	//******************first cloud******************************//
	// initialize PointClouds
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
//	std::string file = RANSACSORCESDIR + std::string("/points.txt");
//	pcl::ModelCoefficients k;
//
//	fitline(file, final, k);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr final_2 (new pcl::PointCloud<pcl::PointXYZ>);
//
//	std::string file_2 = RANSACSORCESDIR + std::string("/points2.txt");
//	pcl::ModelCoefficients k_2;
//
//	fitline(file_2, final_2, k_2);
//	Eigen::Vector4f intersect_pt;
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr intersect (new pcl::PointCloud<pcl::PointXYZ>);
//	find_intersection(k, k_2, intersect_pt, intersect);
//
//


	return 0;
}
