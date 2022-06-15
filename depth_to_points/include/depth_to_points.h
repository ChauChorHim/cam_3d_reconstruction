#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <vector>
#include <cstring>

#include <pcl/impl/point_types.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "npy.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

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
    void depthToPointCloud(PointCloudT& cur_pc, std::string& path_to_depth_npy, std::string& path_to_image, std::string& path_to_mask);
    void addPointCloud(PointCloudT &cur_pc);
    void assignPointCloud(PointCloudT &cur_pc);
    void removeOutlier();
    void downSample();
    void save(std::string& path_to_pcd_file);
    void clear();

private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    PointCloudT cloud_;
    pcl::StatisticalOutlierRemoval<PointT> outlier_sort_;
    pcl::VoxelGrid<PointT> downsample_sort_;    
};

};

#endif
