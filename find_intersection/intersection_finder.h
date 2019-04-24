//
// Created by fongsu on 4/22/19.
//

#ifndef PCL_REAL_INTERSECTION_FINDER_H
#define PCL_REAL_INTERSECTION_FINDER_H

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

void
fitline(std::string file, pcl::PointCloud<pcl::PointXYZ>::Ptr result, pcl::ModelCoefficients &coeffs, float threshold) {
	//******************first cloud******************************//
	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = 100;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	//std::string file = RANSACSORCESDIR + std::string("/points.txt"); // read cloud from file
	std::cout << file << std::endl;
	readCoords(cloud, file); //tranfer the data from the file to a pointcloud object
	// populate our PointCloud with points

	std::vector<int> inliers; // a vector for storing the indexes of the inliers of the model

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
			model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> rans(model_l); // create ransac object for the target cloud
	rans.setDistanceThreshold(
			threshold); //a point is a inlier if the distance between it the fitted line is less than the set value,
	rans.computeModel();
	rans.getInliers(inliers); // get the indexes of the inliers with respect to the original cloud
	Eigen::VectorXf mcoeff;
	rans.getModelCoefficients(mcoeff);// get the coefficients of the fitted line/model, which is the two points that the
	// fitted line passes through in this case, thus capable to form the two-point
	// form of a straight line

	for (int i = 0; i != mcoeff.size(); i++) {
		double j = mcoeff[i];
		coeffs.values.push_back(j);
		std::cout << j << std::endl;
	}

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *result);
}

void find_intersection(pcl::ModelCoefficients &l1, pcl::ModelCoefficients &l2, Eigen::Vector4f &intersection,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_cloud) {
	pcl::lineWithLineIntersection(l1, l2, intersection);
	pcl::PointXYZ p;
	p.x = intersection[0];
	p.y = intersection[1];
	p.z = intersection[2];
	intersection_cloud->push_back(p);
}

#endif //PCL_REAL_INTERSECTION_FINDER_H
