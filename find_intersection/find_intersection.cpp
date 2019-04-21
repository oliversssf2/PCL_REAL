//
// Created by fongsu on 4/21/19.
//
#include <iostream>
#include <thread>

#include <cam_util.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>

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

	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width    = 100;
	cloud->height   = 1;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);

	std::string file = "/home/fongsu/workspace/PCL_REAL/find_intersection/points.txt";
	readCoords(cloud, file);
	// populate our PointCloud with points

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
			model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> rans(model_l);
	rans.setDistanceThreshold(.12);
	rans.computeModel();
	rans.getInliers(inliers);
	Eigen::VectorXf model_coefficients;
	rans.getModelCoefficients(model_coefficients);
	pcl::ModelCoefficients k;
	for(int i = 0; i != model_coefficients.size(); i++)
	{
		double j =  model_coefficients[i];
		k.values.push_back(j);
		std::cout << k << std::endl;
	}
	std::cout << model_coefficients << std::endl;
	std::cout << model_coefficients.value() << std::endl;
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

	// creates the visualization object and adds either our original cloud or all of the inliers
	// depending on the command line arguments specified.
	pcl::visualization::PCLVisualizer::Ptr viewer;
//	if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
//		viewer = simpleVis(final);
//	else
//		viewer = simpleVis(cloud);
//	viewer = simpleVis(cloud);
//	viewer->addCoordinateSystem (1.0);
//	viewer->spin();
	//std::this_thread::sleep_for(1000ms);

	viewer = simpleVis(final);
	viewer->addLine(k, "line one");
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		std::this_thread::sleep_for(100ms);
	}
	return 0;
}
