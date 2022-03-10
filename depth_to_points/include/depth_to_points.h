#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/highgui.hpp>


class BackProjection {
public:
    BackProjection(int height, int width);

    void backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points, std::vector<int>& indices_to_keep);
    void backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points);

private:
    int height_;
    int width_;
    Eigen::Matrix3Xd coordinates_;
    void npy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out);
};

class PCDSaver {
public:
    PCDSaver(int height, int width, Eigen::Matrix3d& K);
    void save(std::string& path_to_pcd_file);
    void addDepthMap(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    void depth2rgb(std::string& path_to_depth_npy, std::string& path_to_image);
private:
    BackProjection backProjector_;
    Eigen::Matrix3d K_inv_;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_;

    void hsv2rgb(int hue, int saturation, int value, int* r, int* g, int* b);
};

class PointCloudSaver {
public:
    PointCloudSaver(float fx, float fy, float cx, float cy);
    void addPointCloud(std::string& path_to_depth_npy, std::string& path_to_image);
private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    template<class vecType>
    void npy2vec(std::string data_fname, std::vector<vecType> &data_out);
};

#endif