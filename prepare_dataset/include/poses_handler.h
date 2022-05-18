#ifndef __POSES_HANDLER__
#define __POSES_HANDLER__

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

namespace cch {

/*
    Given R1, t1 and R2, t2 in the same coordinates, compute the R and t between q1 and q2 -> q2 = R2^{-1} R1 q1 + R2^{-1} (t1 - t2)
*/
void getRelativePose(
    const Eigen::Vector3d &source_pos, const Eigen::Vector3d &target_pos, 
    const Eigen::Quaterniond &source_q, const Eigen::Quaterniond &target_q,
    Eigen::Vector3d &relative_pos, Eigen::Quaterniond &relative_q);

void getAbsolutePose(
    const Eigen::Vector3d &pre_pos, const Eigen::Vector3d &cur_pos,
    const Eigen::Quaterniond &pre_q, const Eigen::Quaterniond &cur_q,
    Eigen::Vector3d &cur_abs_pos, Eigen::Quaterniond &cur_abs_q);

/* --------------------------------------------------------------------------------- */

void getRelativePose(
    const Eigen::Vector3d &source_pos, const Eigen::Vector3d &target_pos, 
    const Eigen::Quaterniond &source_q, const Eigen::Quaterniond &target_q,
    Eigen::Vector3d &relative_pos, Eigen::Quaterniond &relative_q) {

    relative_q = source_q.inverse() * target_q;
    relative_pos = source_q.inverse() * (target_pos - source_pos);
}

void getAbsolutePose(
    const Eigen::Vector3d &pre_pos, const Eigen::Vector3d &cur_pos,
    const Eigen::Quaterniond &pre_q, const Eigen::Quaterniond &cur_q,
    Eigen::Vector3d &cur_abs_pos, Eigen::Quaterniond &cur_abs_q) {

    cur_abs_q = cur_q * pre_q;
    cur_abs_pos = cur_q * pre_pos + cur_pos;
}

};

#endif