#ifndef __POSES_HANDLER__
#define __POSES_HANDLER__

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

namespace cch {

class PosesHandler {

public:
    explicit PosesHandler();
    void getRelativePose(
        const Eigen::Vector3d &source_pos, const Eigen::Vector3d &target_pos, 
        const Eigen::Quaterniond &source_q, const Eigen::Quaterniond &target_q,
        Eigen::Vector3d &relative_pos, Eigen::Quaterniond &relative_q);
    // void getAbsolutePose(
    //     const std::string &timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q);

    // void setInitPose(Eigen::Vector3d& pos_init, Eigen::Quaterniond& q_init);
    // void getInitPose(Eigen::Vector3d& pos_init, Eigen::Quaterniond& q_init);

private:
    // Eigen::Vector3d pos_init_;
    // Eigen::Quaterniond q_init_;
    // Eigen::Quaterniond q_init_inv_;
};

PosesHandler::PosesHandler() {}

void PosesHandler::getRelativePose(
    const Eigen::Vector3d &source_pos, const Eigen::Vector3d &target_pos, 
    const Eigen::Quaterniond &source_q, const Eigen::Quaterniond &target_q,
    Eigen::Vector3d &relative_pos, Eigen::Quaterniond &relative_q) {

    relative_q = target_q.inverse() * source_q;
    relative_pos = target_q.inverse() * (source_pos - target_pos);
}

};

#endif