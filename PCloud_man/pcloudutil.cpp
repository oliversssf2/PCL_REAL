//
// Created by fongsu on 3/15/19.
//

#include "pcloudutil.h"

#include <boost/make_shared.hpp>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/point_cloud.h>
#include <pcl-1.9/pcl/point_representation.h>

#include<pcl-1.9/pcl/io/pcd_io.h>

#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl-1.9/pcl/registration/icp.h>
#include <pcl-1.9/pcl/registration/icp_nl.h>
#include <pcl-1.9/pcl/registration/transforms.h>

#include <pcl-1.9/pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new PointCloud){};
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};


class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        out[0] = p.x;
        out[1] = p.y;
        out[3] = p.z;
        out[4] = p.curvature;
    }
};

void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud ("vp1_target");
    p->removePointCloud ("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO ("Press q to begin the registration.\n");
    p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud ("source");
    p->removePointCloud ("target");


    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
    if (!tgt_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
    if (!src_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");


    p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce();
}

void loadData (int argc, char**argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > *models)
{
    std::string extension (".pcd");

    for(int i = 1; i < argc; i++)
    {
        for (int i = 1; i < argc; i++)
        {
            std::string fname = std::string(argv[i]);

            if(fname.size () <= extension.size())
                continue;

            std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

            if(fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
            {
                PCD m;
                m.f_name = argv[i];
                pcl::io::loadPCDFile(argv[i], *m.cloud);
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

                models->push_back(m);
            }
        }
    }
}

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if(downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    } else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    MyPointRepresentation point_representation;

    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues(alpha);

    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationRotationEpsilon(1e-6);
    reg.setMaxCorrespondenceDistance(0.1);
    //reg.setPointRepresentation()
}
