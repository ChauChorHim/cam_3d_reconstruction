#ifndef __ROTATION_HANDLER__
#define __ROTATION_HANDLER__

#include <Eigen/Dense>

namespace cch {

void matrix2PoseAndQ(Eigen::Matrix<double, 3, 4>& transform_matrix, Eigen::Vector3d& pos, Eigen::Quaterniond& q);

};

#endif
