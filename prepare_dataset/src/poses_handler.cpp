#include "poses_handler.h"

PosesHandler::PosesHandler() {}

void PosesHandler::getRelativePose(
    const Eigen::Vector3d &source_pos, const Eigen::Vector3d &target_pos, 
    const Eigen::Quaterniond &source_q, const Eigen::Quaterniond &target_q,
    Eigen::Vector3d &relative_pos, Eigen::Quaterniond &relative_q) {

    relative_q = source_q.inverse() * target_q;
    relative_pos = source_q.inverse() * (target_pos - source_pos);
}

// void PosesHandler::getAbsolutePose(
//     const std::string &timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    
//     gps_to_pose_.timestamp2Pose(timestamp, pos, q);
// }

// void PosesHandler::setInitPose(Eigen::Vector3d& pos_init, Eigen::Quaterniond& q_init) {
//     pos_init_ = pos_init;
//     q_init_ = q_init;
//     q_init_inv_ = q_init_.inverse();
// }

// void PosesHandler::getInitPose(Eigen::Vector3d& pos_init, Eigen::Quaterniond& q_init) {
//     pos_init = pos_init_;
//     q_init = q_init_;
// }