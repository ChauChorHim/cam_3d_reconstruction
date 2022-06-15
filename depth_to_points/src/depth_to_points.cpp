#include <iostream>
#include <sstream>
#include <iomanip>

#include "depth_to_points.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>

namespace cch {


PointCloudSaver::PointCloudSaver(float fx, float fy, float cx, float cy) : fx_{fx}, fy_{fy}, cx_{cx}, cy_{cy} {
    cloud_.resize(0); 
}

void PointCloudSaver::depthToPointCloud(PointCloudT& cur_pc, std::string& path_to_depth_npy, std::string& path_to_image, std::string& path_to_mask) {
    std::vector<float> depth_vec;
    npy2vec<float>(path_to_depth_npy, depth_vec);

    std::cout << "Min/Max depth: " 
              << *std::min_element(depth_vec.begin(), depth_vec.end()) << " " 
              << *std::max_element(depth_vec.begin(), depth_vec.end()) << "\n";

    cv::Mat image_mat = cv::imread(path_to_image);
    int data_row = image_mat.rows;
    int data_col = image_mat.cols;

    bool use_mask = ("" != path_to_mask);
    cv::Mat1b mask_mat;
    if (use_mask)
        mask_mat = cv::imread(path_to_mask, cv::IMREAD_GRAYSCALE);

    cur_pc.clear();
    cur_pc.height = data_row;
    cur_pc.width = data_col;
    cur_pc.points.resize(data_row * data_col);

    {
        PointT pt;
        pt.x = pt.y = pt.z = 0;
        pt.b = pt.g = pt.r = 0;
        pt.a = 255;
        cur_pc.points.assign(cur_pc.size(), pt);
    }

    for (unsigned idx = 0; idx < data_row * data_col; ++idx) {
        unsigned v = idx / data_col;
        unsigned u = idx % data_col;

        PointT &pt = cur_pc[idx];

        if (use_mask) {
            if (mask_mat.at<uchar>(v, u) != 0) {
                depth_vec[idx] = 0;
            }
        }

        float pixel_depth = depth_vec[idx];

        /* filter out some points */
        if (pixel_depth < 10 && pixel_depth > 5) {
            pt.z = pixel_depth;
            pt.x = (static_cast<float> (u) - cx_) / fx_* pt.z;
            pt.y = (static_cast<float> (v) - cy_) / fy_* pt.z;

            cv::Vec3b pixel_image = image_mat.at<cv::Vec3b>(v, u);

            pt.r = pixel_image[2];
            pt.g = pixel_image[1];
            pt.b = pixel_image[0];
        }
    }
}

void PointCloudSaver::addPointCloud(PointCloudT &new_pc) {
    cloud_ = cloud_ + new_pc;
}

void PointCloudSaver::assignPointCloud(PointCloudT &new_pc) {
    cloud_ = std::move(new_pc);
}

void PointCloudSaver::removeOutlier() {
    // refer to http://pointclouds.org/documentation/tutorials/statistical_outlier.html#statistical-outlier-removal
    PointCloudT::ConstPtr pcloud = cloud_.makeShared();
    PointCloudT::Ptr cloud_filtered (new PointCloudT);

    outlier_sort_.setInputCloud(pcloud);
    outlier_sort_.setMeanK(50);
    outlier_sort_.setStddevMulThresh (1.0);
    outlier_sort_.filter (*cloud_filtered);
    cloud_ = std::move(*cloud_filtered);
}

void PointCloudSaver::downSample() {
    // refer to http://pointclouds.org/documentation/tutorials/voxel_grid.html#voxelgrid
    PointCloudT::ConstPtr pcloud = cloud_.makeShared();
    PointCloudT::Ptr cloud_filtered (new PointCloudT);
    downsample_sort_.setInputCloud(pcloud);
    downsample_sort_.setLeafSize (0.1f, 0.1f, 0.1f);
    downsample_sort_.filter (*cloud_filtered);
    cloud_ = std::move(*cloud_filtered);
}

void PointCloudSaver::save(std::string& path_to_pcd_file) {
    std::cout << "pointcloud size: " << cloud_.size() << "\n";
    pcl::io::savePCDFileASCII (path_to_pcd_file, cloud_);
}

void PointCloudSaver::clear() {
    cloud_.clear();
}

};
