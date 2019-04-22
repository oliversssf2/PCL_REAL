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
	//******************first cloud******************************//
	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width    = 100;
	cloud->height   = 1;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);

	std::string file = RANSACSORCESDIR + std::string("/points.txt"); // read cloud from file
	std::cout << file << std::endl;
	readCoords(cloud, file); //tranfer the data from the file to a pointcloud object
	// populate our PointCloud with points

	std::vector<int> inliers; // a vector for storing the indexes of the inliers of the model

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
			model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> rans(model_l); // create ransac object for the target cloud
	rans.setDistanceThreshold(.12); //a point is a inlier if the distance between it the fitted line is less than the set value,
	rans.computeModel();
	rans.getInliers(inliers); // get the indexes of the inliers with respect to the original cloud
	Eigen::VectorXf mcoeff;
	rans.getModelCoefficients(mcoeff);// get the coefficients of the fitted line/model, which is the two points that the
												// fitted line passes through in this case, thus capable to form the two-point
												// form of a straight line

	pcl::ModelCoefficients k;
	std::vector<int> model;
	rans.getModel(model);
	//for(auto &k : model) std::cout << k << std::endl;

	for(int i = 0; i != mcoeff.size(); i++)
	{
		double j =  mcoeff[i];
		k.values.push_back(j);
		std::cout << j << std::endl;
	}

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

	//******************first cloud******************************//
	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_2 (new pcl::PointCloud<pcl::PointXYZ>);

	cloud_2->width    = 100;
	cloud_2->height   = 1;
	cloud_2->is_dense = false;
	cloud_2->points.resize (cloud->width * cloud->height);

	std::string file_2 = RANSACSORCESDIR + std::string("/points2.txt"); // read cloud from file
	std::cout << file_2 << std::endl;
	readCoords(cloud_2, file_2); //tranfer the data from the file to a pointcloud object
	// populate our PointCloud with points

	std::vector<int> inliers_2; // a vector for storing the indexes of the inliers of the model

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
			model_l_2 (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud_2));
	pcl::RandomSampleConsensus<pcl::PointXYZ> rans_2(model_l_2); // create ransac object for the target cloud
	rans_2.setDistanceThreshold(.50); //a point is a inlier if the distance between it the fitted line is less than the set value,
	rans_2.computeModel();
	rans_2.getInliers(inliers_2); // get the indexes of the inliers with respect to the original cloud
	Eigen::VectorXf mcoeff_2;
	rans_2.getModelCoefficients(mcoeff_2);// get the coefficients of the fitted line/model, which is the two points that the
	// fitted line passes through in this case, thus capable to form the two-point
	// form of a straight line

	pcl::ModelCoefficients k_2;
	std::vector<int> model_2;
	rans_2.getModel(model_2);
	//for(auto &k : model) std::cout << k << std::endl;

	for(int i = 0; i != mcoeff_2.size(); i++)
	{
		double j =  mcoeff_2[i];
		k_2.values.push_back(j);
		std::cout << j << std::endl;
	}

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_2, inliers_2, *final_2);


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
	viewer->addPointCloud(final_2);
	viewer->addCoordinateSystem();

	auto k_ex = k;
	auto k_ex_2 = k_2;

	k_ex.values[0] = k.values[0] + ((k.values[3]-k.values[0])*10);
	k_ex.values[1] = k.values[1] + ((k.values[4]-k.values[1])*10);
	k_ex.values[3] = k.values[3] + ((k.values[3]-k.values[0])*-10);
	k_ex.values[4] = k.values[4] + ((k.values[4]-k.values[1])*-10);

	k_ex_2.values[0] = k_2.values[0] + ((k_2.values[3]-k_2.values[0])*10);
	k_ex_2.values[1] = k_2.values[1] + ((k_2.values[4]-k_2.values[1])*10);
	k_ex_2.values[3] = k_2.values[3] + ((k_2.values[3]-k_2.values[0])*-10);
	k_ex_2.values[4] = k_2.values[4] + ((k_2.values[4]-k_2.values[1])*-10);

	viewer->addLine(k_ex, "line one");
	viewer->addLine(k_ex_2, "line two");
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		std::this_thread::sleep_for(100ms);
	}
	return 0;
}
