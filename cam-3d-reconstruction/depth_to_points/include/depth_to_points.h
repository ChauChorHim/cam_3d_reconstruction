#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <Eigen/Dense>

#include <vector>
#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

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
    void depthToPointCloud(std::string& path_to_depth_npy, std::string* path_to_image=nullptr);
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
    
    void depthToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cur_cloud, std::string& path_to_depth_npy, std::string* path_to_image);
    // void depth2rgb(std::string& path_to_depth_npy, std::string& path_to_image);
    // void hsv2rgb(int hue, int saturation, int value, int* r, int* g, int* b);
};

/* --------------------------------------------------------------------------------- */

PointCloudSaver::PointCloudSaver(float fx, float fy, float cx, float cy) : fx_{fx}, fy_{fy}, cx_{cx}, cy_{cy} {
    cloud_.resize(0);
    
}

void PointCloudSaver::depthToPointCloud(std::string& path_to_depth_npy, std::string* path_to_image) {
    pcl::PointCloud<pcl::PointXYZRGB> cur_cloud_camera;
    depthToPointCloud(cur_cloud_camera, path_to_depth_npy, path_to_image);
    cloud_ = std::move(cur_cloud_camera);
}

void PointCloudSaver::depthToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cur_cloud, std::string& path_to_depth_npy, std::string* path_to_image) {
    if (path_to_image == nullptr) {
        throw NotImplemented();
    }

    std::vector<float> depth_vec;
    npy2vec<float>(path_to_depth_npy, depth_vec);

    std::cout << "Min/Max depth: " 
              << *std::min_element(depth_vec.begin(), depth_vec.end()) << " " 
              << *std::max_element(depth_vec.begin(), depth_vec.end()) << "\n";

    cv::Mat image_mat = cv::imread(*path_to_image);
    int data_row = image_mat.rows;
    int data_col = image_mat.cols;

    // Clear the pointcloud
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

        float pixel_depth = depth_vec[idx];

        /* filter out some points */
        if (pixel_depth < 30 && pixel_depth > 5) {
            pt.z = pixel_depth;
            pt.x = (static_cast<float> (u) - cx_) / fx_* pt.z;
            pt.y = (static_cast<float> (v) - cy_) / fy_* pt.z;

            cv::Vec3b pixel_image = image_mat.at<cv::Vec3b>(idx / data_col, idx % data_col);

            pt.r = pixel_image[2];
            pt.g = pixel_image[1];
            pt.b = pixel_image[0];
        }
    }
}

void PointCloudSaver::addPointCloudWithPose(pcl::PointCloud<pcl::PointXYZRGB> &cur_pc_cam, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    Eigen::Matrix4d trans;
    trans.setIdentity();
    q = q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
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

// void PointCloudSaver::depth2rgb(std::string& path_to_depth_npy, std::string& path_to_image) {
//     double min_intensity = 255.f;
//     double max_intensity = 0.f;

//     Eigen::Matrix3Xd camera_points;
//     backProjector_.backProjec(path_to_depth_npy, K_inv_, camera_points);

//     for (const auto& pt : cloud.points) {
//         Eigen::Vector3d img_point_depth;
//         Eigen::Vector3d p(pt.x + lidar2car_x, pt.y + lidar2car_y, pt.z);
//         bool flag = aw::coord::LiDARPoint2ImagePointWithDepth(p, cfg, &img_point_depth);

//         if (!flag || img_point_depth(2) <= ignore_depth) {
//             continue;
//         }

//         double intensity = pt.intensity;

//         if (intensity > max_intensity) {
//             max_intensity = intensity;
//         }

//         if (intensity < min_intensity) {
//             min_intensity = intensity;
//         }

//         Eigen::Vector3d uvi(img_point_depth(0), img_point_depth(1), intensity);
//         corrs_points_vec.push_back(uvi);
//     }

//     for (const auto& uvi : corrs_points_vec) {
//         int u = static_cast<int>(uvi(0));
//         int v = static_cast<int>(uvi(1));
//         int i = static_cast<int>(uvi(2));

//         double color_i = (i - min_intensity) / (max_intensity - min_intensity);
//         int hue = static_cast<int>((color_i)*360);

//         int saturation = 1;
//         int value = 1;

//         int r = 0, g = 0, b = 0;
//         hsv2rgb(hue, saturation, value, &r, &g, &b);

//         cv::circle(*img, cv::Point(u, v), 1, cv::Scalar(r, g, b), 2);
//     }
// }

// void PointCloudSaver::hsv2rgb(int hue, int saturation, int value, int* r, int* g, int* b) {
//     int c = value * saturation;
//     int x = c * (1 - std::fabs((hue / 60) % 2 - 1));
//     int m = value - c;

//     int r_prime = 0, g_prime = 0, b_prime = 0;
//     if (hue >= 0 and hue < 60) {
//         r_prime = c;
//         g_prime = x;
//         b_prime = 0;
//     } else if (hue >= 60 and hue < 120) {
//         r_prime = x;
//         g_prime = c;
//         b_prime = 0;
//     } else if (hue >= 120 and hue < 180) {
//         r_prime = 0;
//         g_prime = c;
//         b_prime = x;
//     } else if (hue >= 180 and hue < 240) {
//         r_prime = 0;
//         g_prime = x;
//         b_prime = c;
//     } else if (hue >= 240 and hue < 300) {
//         r_prime = x;
//         g_prime = 0;
//         b_prime = c;
//     } else if (hue >= 300 and hue < 360) {
//         r_prime = c;
//         g_prime = 0;
//         b_prime = x;
//     }

//     *r = static_cast<int>((r_prime + m) * 255);
//     *g = static_cast<int>((g_prime + m) * 255);
//     *b = static_cast<int>((b_prime + m) * 255);

//     return;
// }

};

#endif
