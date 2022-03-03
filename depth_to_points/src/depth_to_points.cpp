#include "depth_to_points.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "cnpy.h"

void cnpy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out){
    cnpy::NpyArray npy_data = cnpy::npy_load(data_fname);
    // double* ptr = npy_data.data<double>();
    int data_row = npy_data.shape[0];
    int data_col = npy_data.shape[1];
    double* ptr = static_cast<double *>(malloc(data_row * data_col * sizeof(double)));
    memcpy(ptr, npy_data.data<double>(), data_row * data_col * sizeof(double));
    cv::Mat dmat = cv::Mat(cv::Size(data_col, data_row), CV_64F, ptr); // CV_64F is equivalent double
    new (&mat_out) Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>>(reinterpret_cast<double *>(dmat.data), data_col, data_row);
}

BackProjection::BackProjection(int height, int width) : height_ {height}, width_ {width} {
    coordinates_ = Eigen::Matrix3Xd (3, height_ * width_);
    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            coordinates_(0, row * width_ + col) = row;
            coordinates_(1, row * width_ + col) = col;
            coordinates_(1, row * width_ + col) = 1;
        }
    }
    
}

void BackProjection::backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points) {
    Eigen::Matrix3Xd points_camera_norm = inv_K * coordinates_;

    Eigen::MatrixXd depth_eigen;
    cnpy2eigen(path_to_depth_npy, depth_eigen);

    int rows = depth_eigen.rows();
    int cols = depth_eigen.cols();
    std::vector<double> depth_element;

    for (int i = 0; i < 3; ++i) {
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                depth_element.push_back(depth_eigen(row, col));
            }
        }
    }
    camera_points = Eigen::Matrix3Xd::Map(&depth_element[0], 3, rows * cols);
}


PCDSaver::PCDSaver(int height, int width, double min_depth, double max_depth, Eigen::Matrix3d& K) : 
    backProjector_ {BackProjection(height, width)}, min_depth_ {min_depth}, max_depth_ {max_depth}, K_inv_ {K.inverse()} {

}

void PCDSaver::save(std::string& path_to_pcd_file) {
    pcl::io::savePCDFileASCII (path_to_pcd_file, *cloud);
}

void PCDSaver::addDepthMap(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q_inv) {
    
    cv::Mat image = cv::imread(path_to_image, cv::IMREAD_COLOR);
    cv::Mat rgb_data = image.reshape(3, 1); // 1 x image.rows * image.cols x 3

    Eigen::Matrix3Xd camera_points;
    backProjector_.backProjec(path_to_depth_npy, K_inv_, camera_points);
    
    Eigen::Matrix3Xd world_points;
    world_points = q_inv.toRotationMatrix() * camera_points + pos;

    pcl::PointXYZRGB point;

    for (size_t i = 0; i < world_points.cols(); ++i) {
        point.x = world_points(0, i);
        point.y = world_points(1, i);
        point.z = world_points(2, i);
        std::uint8_t r = rgb_data.at<double>(2, i);
        std::uint8_t g = rgb_data.at<double>(1, i);
        std::uint8_t b = rgb_data.at<double>(0, i);
        std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
        point.rgb = *reinterpret_cast<float*>(&rgb);

        // push back point to point cloud one by one
        cloud->push_back(point);
    }

}

