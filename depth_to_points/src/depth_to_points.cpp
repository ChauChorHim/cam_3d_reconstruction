#include "depth_to_points.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "cnpy.h"

void cnpy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out){
    cnpy::NpyArray npy_data = cnpy::npy_load(data_fname);
    int data_row = npy_data.shape[0];
    int data_col = npy_data.shape[1];

    double* ptr = static_cast<double *>(malloc(data_row * data_col * sizeof(double)));
    memcpy(ptr, npy_data.data<double>(), data_row * data_col * sizeof(double));
    cv::Mat dmat = cv::Mat(cv::Size(data_row, data_col), CV_64F, ptr); // CV_64F is equivalent double
    new (&mat_out) Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>>(reinterpret_cast<double *>(dmat.data), data_row, data_col);
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

void BackProjection::backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points, std::vector<int>& indices_to_keep) {
    Eigen::Matrix3Xd camera_points_norm = inv_K * coordinates_;

    Eigen::MatrixXd depth_eigen;
    cnpy2eigen(path_to_depth_npy, depth_eigen);

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

