#ifndef __POSES_HANDLER__
#define __POSES_HANDLER__

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

namespace cch {


void getRelativePose(
    const Eigen::Vector3d &t1, const Eigen::Vector3d &t2, 
    const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2,
    Eigen::Vector3d &t12, Eigen::Quaterniond &q12);

void getAbsolutePose(
    const Eigen::Vector3d &pre_pos, const Eigen::Vector3d &cur_pos,
    const Eigen::Quaterniond &pre_q, const Eigen::Quaterniond &cur_q,
    Eigen::Vector3d &cur_abs_pos, Eigen::Quaterniond &cur_abs_q);

/* --------------------------------------------------------------------------------- */

void getRelativePose(
    /*
        Given q1, t1 and q2, t2 in the same coordinates (world), compute the R and t between P1 and P2 -> P2 = q2^{-1} q1 P1 + q2^{-1} (t1 - t2)
        refer to https://math.stackexchange.com/questions/709622/relative-camera-matrix-pose-from-global-camera-matrixes
    */
    const Eigen::Vector3d &t1, const Eigen::Vector3d &t2, 
    const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2,
    Eigen::Vector3d &t12, Eigen::Quaterniond &q12) {

    q12 = q2.inverse() * q1;
    t12 = q2.inverse() * (t1 - t2);
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