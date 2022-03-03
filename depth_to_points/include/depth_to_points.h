#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>


class BackProjection {
public:
    BackProjection(int height, int width);

    void backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points);

private:
    int height_;
    int width_;
    Eigen::Matrix3Xd coordinates_;
};

class PCDSaver {
public:
    PCDSaver(int height, int width, double min_depth, double max_depth, Eigen::Matrix3d& K);
    void save(std::string& path_to_pcd_file);
    void addDepthMap(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q_inv);
private:
    BackProjection backProjector_;
    double min_depth_;
    double max_depth_;
    Eigen::Matrix3d K_inv_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    void cnpy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out);
};

#endif