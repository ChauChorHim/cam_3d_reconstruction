#include <iostream>
#include <sstream>
#include <iomanip>

#include "depth_to_points.h"

#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

namespace cch {


PointCloudSaver::PointCloudSaver(float fx, float fy, float cx, float cy) : fx_{fx}, fy_{fy}, cx_{cx}, cy_{cy} {
    cloud_.resize(0);
    
}

void PointCloudSaver::depthToPointCloud(std::string& path_to_depth_npy, std::string& path_to_image, std::string& path_to_mask) {
    pcl::PointCloud<pcl::PointXYZRGB> cur_cloud_camera;
    depthToPointCloud(cur_cloud_camera, path_to_depth_npy, path_to_image, path_to_mask);
    cloud_ = std::move(cur_cloud_camera);
}

void PointCloudSaver::depthToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cur_cloud, std::string& path_to_depth_npy, std::string& path_to_image, std::string& path_to_mask) {

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

    cur_cloud.clear();
    cur_cloud.height = data_row;
    cur_cloud.width = data_col;
    cur_cloud.points.resize(data_row * data_col);

    {
        pcl::PointXYZRGB pt;
        pt.x = pt.y = pt.z = 0;
        pt.b = pt.g = pt.r = 0;
        pt.a = 255;
        cur_cloud.points.assign(cur_cloud.size(), pt);
    }
    
    for (unsigned idx = 0; idx < data_row * data_col; ++idx) {
        unsigned v = idx / data_col;
        unsigned u = idx % data_col;

        pcl::PointXYZRGB &pt = cur_cloud[idx];

        if (use_mask) {
            if (mask_mat.at<uchar>(v, u) != 0) {
                depth_vec[idx] = 0;
            }
        }

        float pixel_depth = depth_vec[idx];

        /* filter out some points */
        if (pixel_depth < 30 && pixel_depth > 5) {
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

void PointCloudSaver::addPointCloudWithPose(pcl::PointCloud<pcl::PointXYZRGB> &cur_pc_cam, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    Eigen::Matrix4d trans;
    trans.setIdentity();
    trans.block<3, 3>(0, 0) = q.toRotationMatrix();
    trans.block<3, 1>(0, 3) = pos;

    pcl::transformPointCloud(cur_pc_cam, cur_pc_cam, trans);

    cloud_ = cloud_ + cur_pc_cam;
}


void PointCloudSaver::removeOutlier() {
    // refer to http://pointclouds.org/documentation/tutorials/statistical_outlier.html#statistical-outlier-removal
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcloud = cloud_.makeShared();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    outlier_sort_.setInputCloud(pcloud);
    outlier_sort_.setMeanK(50);
    outlier_sort_.setStddevMulThresh (1.0);
    outlier_sort_.filter (*cloud_filtered);
    cloud_ = std::move(*cloud_filtered);
}

void PointCloudSaver::downSample() {
    // refer to http://pointclouds.org/documentation/tutorials/voxel_grid.html#voxelgrid
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcloud = cloud_.makeShared();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
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
