#include "depth_to_points.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/transforms.h>

#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <set>
#include <filesystem>

#include "npy.h"


PointCloudSaver::PointCloudSaver(float fx, float fy, float cx, float cy) : fx_{fx}, fy_{fy}, cx_{cx}, cy_{cy} {
    cloud_.resize(0);
}

template<class vecType>
void PointCloudSaver::npy2vec(std::string data_fname, std::vector<vecType> &data_out) {
    std::vector<unsigned long> shape;
    bool is_fortran_order;

    shape.clear();
    data_out.clear();
    npy::LoadArrayFromNumpy(data_fname, shape, is_fortran_order, data_out);

    assert(false == is_fortran_order);
}

template<class pcType>
void PointCloudSaver::loadOnePointCloud(std::string& path_to_depth_npy, std::string& path_to_image, pcl::PointCloud<pcType>& cur_cloud) {
    cv::Mat image_mat = cv::imread(path_to_image);

    std::vector<float> depth_vec;
    npy2vec<float>(path_to_depth_npy, depth_vec);

    std::cout << "Min/Max depth in this point cloud: " << *std::min_element(depth_vec.begin(), depth_vec.end()) << " " << *std::max_element(depth_vec.begin(), depth_vec.end()) << "\n";

    int data_row = image_mat.rows;
    int data_col = image_mat.cols;

    cur_cloud.height = data_row;
    cur_cloud.width = data_col;
    cur_cloud.points.resize(cur_cloud.height * cur_cloud.width);

    {
        pcl::PointXYZRGB pt;
        pt.x = pt.y = pt.z = 0;
        pt.b = pt.g = pt.r = 0;
        pt.a = 255;
        cur_cloud.points.assign(cur_cloud.size(), pt);
    }
    
    unsigned point_idx = 0;
    for (unsigned idx = 0; idx < data_row * data_col; ++idx) {
        unsigned v = idx / data_col;
        unsigned u = idx % data_col;

        pcl::PointXYZRGB &pt = cur_cloud[point_idx];

        float pixel_depth = depth_vec[point_idx];

        /* filter out some points */
        if (pixel_depth < 0.5) {
            pt.z = pixel_depth * 25;
            pt.x = (static_cast<float> (u) - cx_) / fx_* pt.z;
            pt.y = (static_cast<float> (v) - cy_) / fy_* pt.z;

            cv::Vec3b pixel_image = image_mat.at<cv::Vec3b>(point_idx / data_col, point_idx % data_col);

            pt.r = pixel_image[2];
            pt.g = pixel_image[1];
            pt.b = pixel_image[0];
        }

        point_idx++;
    }
}

void PointCloudSaver::saveOnePointCloud(std::string& path_to_depth_npy, std::string& path_to_image, std::string& path_to_pcd_file, bool is_folder) {
    pcl::PointCloud<pcl::PointXYZRGB> cur_cloud;
    if (is_folder == false) {
        // only handle one image
        loadOnePointCloud(path_to_depth_npy, path_to_image, cur_cloud);
        pcl::io::savePCDFileASCII (path_to_pcd_file, cur_cloud);
        cur_cloud.clear();
    } else {
        // handle many images
         std::set<std::filesystem::path> depth_list;
         std::set<std::filesystem::path> image_list;

        for (const auto &path_to_file: std::filesystem::directory_iterator(path_to_depth_npy))
            depth_list.insert(path_to_file.path());

        for (const auto &path_to_file: std::filesystem::directory_iterator(path_to_image))
            image_list.insert(path_to_file.path());

        assert (depth_list.size() == image_list.size());

        std::stringstream ss;
        std::string s;

        std::set<std::filesystem::path>::iterator iter_depth = depth_list.begin();
        std::set<std::filesystem::path>::iterator iter_image = image_list.begin();

        for (size_t i = 0; i < depth_list.size(); ++i) {
            std::string path_to_depth = *iter_depth;
            iter_depth++;
            std::string path_to_image = *iter_image;
            iter_image++;
            loadOnePointCloud(path_to_depth, path_to_image, cur_cloud);

            ss.str("");
            ss << std::setw(6) << std::setfill('0') << i;
            s = path_to_pcd_file + "/" + ss.str() + ".pcd";

            pcl::io::savePCDFileASCII(s, cur_cloud);
            cur_cloud.clear();
        }
    }
    
}

void PointCloudSaver::addPointCloud(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    pcl::PointCloud<pcl::PointXYZRGB> cur_cloud_camera, cur_cloud_world;
    loadOnePointCloud(path_to_depth_npy, path_to_image, cur_cloud_camera);


    Eigen::Matrix4d trans;
    trans.setIdentity();
    Eigen::Quaterniond q_ = q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
    trans.block<3, 3>(0, 0) = q_.toRotationMatrix();
    trans.block<3, 1>(0, 3) = pos;
    pcl::transformPointCloud(cur_cloud_camera, cur_cloud_world, trans);

    cloud_ = cloud_ + cur_cloud_world;
}

void PointCloudSaver::save(std::string& path_to_pcd_file) {
    pcl::io::savePCDFileASCII (path_to_pcd_file, cloud_);
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
