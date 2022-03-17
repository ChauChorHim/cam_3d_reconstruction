#ifndef __POSES_HANDLER__
#define __POSES_HANDLER__

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include "timestamp_to_pose.h"

class PosesHandler {

public:
    explicit PosesHandler();
    void addGpsData(const std::string& path_to_gps_file);
    void getRelativePose(
        const std::string &source_timestamp, const std::string &target_timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    void getAbsolutePose(
        const std::string &timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q);

private:
    std::string path_to_gps_file_;
    Gps2Pose gps_to_pose_;
};

#endif