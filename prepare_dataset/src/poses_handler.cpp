#include "poses_handler.h"

PosesHandler::PosesHandler() {
    gps_to_pose_ = Gps2Pose();
}

void PosesHandler::addGpsData(const std::string& path_to_gps_file) {
    gps_to_pose_.addGpsData(path_to_gps_file);
    gps_to_pose_.initializePose(nullptr);
}

void PosesHandler::getRelativePose(
    const std::string &source_timestamp, const std::string &target_timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    Eigen::Vector3d source_pos, target_pos;
    Eigen::Quaterniond source_q, target_q;
    gps_to_pose_.timestamp2Pose(source_timestamp, source_pos, source_q);
    gps_to_pose_.timestamp2Pose(target_timestamp, target_pos, target_q);
    
    q = source_q.inverse() * target_q;
    pos = source_q.inverse() * (target_pos - source_pos);
}

void PosesHandler::getAbsolutePose(
    const std::string &timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    
    gps_to_pose_.timestamp2Pose(timestamp, pos, q);
}