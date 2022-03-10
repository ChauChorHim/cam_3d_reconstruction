#include "depth_to_points.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "npy.h"
#include <cstring>

void BackProjection::npy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out){
    
}


BackProjection::BackProjection(int height, int width) : height_ {height}, width_ {width} {
    coordinates_ = Eigen::Matrix3Xd (3, height_ * width_);
    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            coordinates_(0, row * width_ + col) = row;
            coordinates_(1, row * width_ + col) = col;
            coordinates_(2, row * width_ + col) = 1;
        }
    }
}



void BackProjection::backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points) {
    Eigen::Matrix3Xd camera_points_norm = inv_K * coordinates_;

    Eigen::MatrixXd depth_eigen;
    npy2eigen(path_to_depth_npy, depth_eigen);

    int rows = depth_eigen.rows();
    int cols = depth_eigen.cols();

    std::vector<double> depth_element;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            for (int i = 0; i < 3; ++i) {
                depth_element.push_back(depth_eigen(row, col) * camera_points_norm(i, row * cols + col));              // push back X, Y, Z of each camera points
            }
        }
    }
    std::cout << depth_element.size() / 3 << " points are appended to point cloud\n\n";

    camera_points = Eigen::Matrix3Xd::Map(&depth_element[0], 3, depth_element.size() / 3);
}

void BackProjection::backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points, std::vector<int>& indices_to_keep) {
    Eigen::Matrix3Xd camera_points_norm = inv_K * coordinates_;

    Eigen::MatrixXd depth_eigen;
    npy2eigen(path_to_depth_npy, depth_eigen);

    int rows = depth_eigen.rows();
    int cols = depth_eigen.cols();

    std::vector<double> depth_element;

    bool isPush = false;
    
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            isPush = false;
            for (int i = 0; i < 3; ++i) {
                if (depth_eigen(row, col) > 2 && depth_eigen(row, col) < 20) {              // Filter out depth not in the range [3, 30]
                    depth_element.push_back(depth_eigen(row, col) * camera_points_norm(i, row * cols + col));              // push back X, Y, Z of each camera points
                    if (!isPush) {
                        indices_to_keep.push_back(row * cols + col);
                        isPush = true;
                    }
                }
            }
        }
    }
    std::cout << depth_element.size() / 3 << " points are appended to point cloud\n\n";

    camera_points = Eigen::Matrix3Xd::Map(&depth_element[0], 3, depth_element.size() / 3);
}


PCDSaver::PCDSaver(int height, int width, Eigen::Matrix3d& K) : 
    backProjector_ {BackProjection(height, width)}, K_inv_ {K.inverse()} {

}

void PCDSaver::save(std::string& path_to_pcd_file) {
    pcl::io::savePCDFileASCII (path_to_pcd_file, cloud_);
}

void PCDSaver::addDepthMap(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {

    Eigen::Matrix3Xd camera_points;
    std::vector<int> indices_to_keep {};
    backProjector_.backProjec(path_to_depth_npy, K_inv_, camera_points, indices_to_keep);

    // Eigen::Quaterniond cam2gps = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) * 
    //                              Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond cam2gps = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * 
                                 Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3Xd world_points = q.toRotationMatrix() * cam2gps.toRotationMatrix() * camera_points;

    // Eigen::Vector3d mean = world_points.rowwise().mean(); 
    // Eigen::Vector3d mean = world_points.rowwise().minCoeff(); 
    // std::cout << mean[0] << " " << mean[1] << " " << mean[2] << "\n";
    // mean = world_points.rowwise().maxCoeff(); 
    // std::cout << mean[0] << " " << mean[1] << " " << mean[2] << "\n\n";

    cv::Mat image = cv::imread(path_to_image);
    pcl::PointXYZRGBA point;
    for (int i = 0; i < world_points.cols(); ++i) {
        if (i % 1) continue;
        point.x = world_points(0, i) - pos[0];
        point.y = world_points(1, i) - pos[1];
        point.z = world_points(2, i) - pos[2];

        int rgb_idx = indices_to_keep[i];

        int row_idx = rgb_idx / image.cols;
        int col_idx = rgb_idx % image.cols;

        cv::Vec3b pixel = image.at<cv::Vec3b>(row_idx, col_idx);

        int blue = pixel.val[0];
        int green = pixel.val[1];
        int red = pixel.val[2];

        point.b = blue;
        point.g = green;
        point.r = red;
        point.a = 0;
        
        cloud_.push_back(point);
    }

    std::cout << "current accumulated cloud number: " << cloud_.width << std::endl;
}

// void PCDSaver::depth2rgb(std::string& path_to_depth_npy, std::string& path_to_image) {
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

void PCDSaver::hsv2rgb(int hue, int saturation, int value, int* r, int* g, int* b) {
    int c = value * saturation;
    int x = c * (1 - std::fabs((hue / 60) % 2 - 1));
    int m = value - c;

    int r_prime = 0, g_prime = 0, b_prime = 0;
    if (hue >= 0 and hue < 60) {
        r_prime = c;
        g_prime = x;
        b_prime = 0;
    } else if (hue >= 60 and hue < 120) {
        r_prime = x;
        g_prime = c;
        b_prime = 0;
    } else if (hue >= 120 and hue < 180) {
        r_prime = 0;
        g_prime = c;
        b_prime = x;
    } else if (hue >= 180 and hue < 240) {
        r_prime = 0;
        g_prime = x;
        b_prime = c;
    } else if (hue >= 240 and hue < 300) {
        r_prime = x;
        g_prime = 0;
        b_prime = c;
    } else if (hue >= 300 and hue < 360) {
        r_prime = c;
        g_prime = 0;
        b_prime = x;
    }

    *r = static_cast<int>((r_prime + m) * 255);
    *g = static_cast<int>((g_prime + m) * 255);
    *b = static_cast<int>((b_prime + m) * 255);

    return;
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

void PointCloudSaver::addPointCloud(std::string& path_to_depth_npy, std::string& path_to_image) {
    cv::Mat image_mat = cv::imread(path_to_image);

    std::vector<float> depth_vec;
    npy2vec<float>(path_to_depth_npy, depth_vec);

    std::cout << "Min/Max depth in this point cloud: " << *std::min_element(depth_vec.begin(), depth_vec.end()) << " " << *std::max_element(depth_vec.begin(), depth_vec.end()) << "\n";

    int data_row = image_mat.rows;
    int data_col = image_mat.cols;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cur_cloud->height = data_row;
    cur_cloud->width = data_col;
    cur_cloud->points.resize(cur_cloud->height * cur_cloud->width);

    {
        pcl::PointXYZRGB pt;
        // pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
        pt.x = pt.y = pt.z = 0;
        pt.b = pt.g = pt.r = 0;
        pt.a = 255;
        cur_cloud->points.assign(cur_cloud->size(), pt);
    }
    

    unsigned point_idx = 0;
    for (unsigned v = 0; v < data_row; ++v) {
        for (unsigned u = 0; u < data_col; ++u) {
            pcl::PointXYZRGB &pt = (*cur_cloud)[point_idx];

            float pixel_depth = depth_vec[point_idx];

            /* filter out some points */
            // if (pixel_depth > 3) 
            //     continue;

            pt.z = pixel_depth;
            pt.x = (static_cast<float> (u) - cx_) / fx_* pt.z;
            pt.y = (static_cast<float> (v) - cy_) / fy_* pt.z;

            cv::Vec3b pixel_image = image_mat.at<uint8_t>(point_idx / data_col, point_idx % data_col);

            std::cout << pixel_image << std::endl;
            pt.r = pixel_image[2];
            pt.g = pixel_image[1];
            pt.b = pixel_image[0];

            // std::cout << pt.r << " " << pt.g << " " << pt.b << std::endl;

            point_idx++;

            // std::cout << pt.x << " " << pt.y << " " << pt.z << "\n";
        }
    }

    std::string path_to_pcd_file {"./test.pcd"};
    pcl::io::savePCDFileASCII (path_to_pcd_file, *cur_cloud);

    return;
    
}


PointCloudSaver::PointCloudSaver(float fx, float fy, float cx, float cy) : fx_{fx}, fy_{fy}, cx_{cx}, cy_{cy} {}