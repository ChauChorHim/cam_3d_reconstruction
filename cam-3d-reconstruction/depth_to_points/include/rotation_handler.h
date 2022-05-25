#ifndef __ROTATION_HANDLER__
#define __ROTATION_HANDLER__

#include <Eigen/Dense>

namespace cch {

void matrix2PoseAndQ(Eigen::Matrix<double, 3, 4>& transform_matrix, Eigen::Vector3d& pos, Eigen::Quaterniond& q);

/* --------------------------------------------------------------------------------- */

void matrix2PoseAndQ(Eigen::Matrix<double, 3, 4>& transform_matrix, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    Eigen::Vector3d pos_tmp;
    pos_tmp << transform_matrix(0, 3), transform_matrix(1, 3), transform_matrix(2, 3);
    pos = std::move(pos_tmp);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << transform_matrix(0, 0), transform_matrix(0, 1), transform_matrix(0, 2), 
                       transform_matrix(1, 0), transform_matrix(1, 1), transform_matrix(1, 2),
                       transform_matrix(2, 0), transform_matrix(2, 1), transform_matrix(2, 2);  
    q = rotation_matrix;
}

};

#endif
