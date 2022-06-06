#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <Eigen/Dense>

#include <vector>
#include <cstring>

#include <pcl/impl/point_types.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "npy.h"

class NotImplemented : public std::logic_error
{
public:
    NotImplemented() : std::logic_error("Function not yet implemented") { };
};

template<class vecType>
void npy2vec(std::string data_fname, std::vector<vecType> &data_out) {
    std::vector<unsigned long> shape;
    bool is_fortran_order;

    shape.clear();
    data_out.clear();
    npy::LoadArrayFromNumpy(data_fname, shape, is_fortran_order, data_out);

    assert(false == is_fortran_order);
}

namespace cch {

class PointCloudSaver {
public:
    PointCloudSaver(float fx, float fy, float cx, float cy);
    void depthToPointCloud(std::string& path_to_depth_npy, std::string& path_to_mask, std::string* path_to_image=nullptr);
    void addPointCloudWithPose(pcl::PointCloud<pcl::PointXYZRGB> &cur_pc_cam, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    void removeOutlier();
    void downSample();
    void save(std::string& path_to_pcd_file);
    void clear();

private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_sort_;
    pcl::VoxelGrid<pcl::PointXYZRGB> downsample_sort_;
    
    void depthToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cur_cloud, std::string& path_to_depth_npy, std::string& path_to_mask, std::string* path_to_image);
};

};

#endif
