#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>


class BackProjection {
public:
    BackProjection(int height, int width);

    void backProjec(std::string& path_to_depth_npy, Eigen::Matrix3d& inv_K, Eigen::Matrix3Xd& camera_points, std::vector<int>& indices_to_keep);

private:
    int height_;
    int width_;
    Eigen::Matrix3Xd coordinates_;
};

class PCDSaver {
public:
    PCDSaver(int height, int width, Eigen::Matrix3d& K);
    void save(std::string& path_to_pcd_file);
    void addDepthMap(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
private:
    BackProjection backProjector_;
    Eigen::Matrix3d K_inv_;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_;

    void cnpy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out);
};

#endif