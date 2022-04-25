#ifndef __ROTATION_HANDLER__
#define __ROTATION_HANDLER__

#include <Eigen/Dense>
#include <string>

namespace cch {

void line2Matrix34(std::string& line, Eigen::Matrix<double, 3, 4>& transform_matrix);
void matrix2PoseAndQ(Eigen::Matrix<double, 3, 4>& transform_matrix, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
void line2PoseAndQ(std::string& line, Eigen::Vector3d& pos, Eigen::Quaterniond& q);

/* --------------------------------------------------------------------------------- */

void line2Matrix34(std::string& line, Eigen::Matrix<double, 3, 4>& transform_matrix) {
    std::istringstream line_stream(line);
    double ele;
    std::vector<double> line_vec;
    for (int i = 0; i < 12; ++i) {
        line_stream >> ele;
        line_vec.push_back(ele);
    }
    transform_matrix << line_vec[0], line_vec[1], line_vec[2], line_vec[3],
                        line_vec[4], line_vec[5], line_vec[6], line_vec[7],
                        line_vec[8], line_vec[9], line_vec[10], line_vec[11];
}

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

void line2PoseAndQ(std::string& line, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    std::istringstream line_stream(line);
    double ele;
    std::vector<double> line_vec;
    for (int i = 1; i < 8; ++i) {
        line_stream >> ele;
        line_vec.push_back(ele);
    }
    pos << line_vec[1], line_vec[2], -line_vec[3];

    q = Eigen::Quaterniond(line_vec[4], line_vec[5], line_vec[6], line_vec[7]);
}

};

#endif
