#ifndef __POSES_HANDLER__
#define __POSES_HANDLER__

#include <eigen3/Eigen/Dense>

namespace cch {

void getRelativePose(
    const Eigen::Vector3d &t1, const Eigen::Vector3d &t2, 
    const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2,
    Eigen::Vector3d &t12, Eigen::Quaterniond &q12);

void getAbsolutePose(
    const Eigen::Vector3d &pre_pos, const Eigen::Vector3d &cur_pos,
    const Eigen::Quaterniond &pre_q, const Eigen::Quaterniond &cur_q,
    Eigen::Vector3d &cur_abs_pos, Eigen::Quaterniond &cur_abs_q);

};

#endif