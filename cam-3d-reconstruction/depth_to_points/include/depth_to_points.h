#ifndef _DEPTH_TO_POINTS_
#define _DEPTH_TP_POINTS_

#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/highgui.hpp>

class PointCloudSaver {
public:
    PointCloudSaver(float fx, float fy, float cx, float cy);
    void saveOnePointCloud(std::string& path_to_depth_npy, std::string& path_to_image, std::string& path_to_pcd_file, bool is_folder=false);
    void addPointCloud(std::string& path_to_depth_npy, std::string& path_to_image, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    void save(std::string& path_to_pcd_file);
    // void depth2rgb(std::string& path_to_depth_npy, std::string& path_to_image);
private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_;

    template<class vecType>
    void npy2vec(std::string data_fname, std::vector<vecType> &data_out);

    template<class pcType>
    void loadOnePointCloud(std::string& path_to_depth_npy, std::string& path_to_image, pcl::PointCloud<pcType>& cur_cloud);

    // void hsv2rgb(int hue, int saturation, int value, int* r, int* g, int* b);
};

#endif