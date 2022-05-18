#include "poses_handler.h"
#include "gps_handler.h"
#include <cmath>
#include <sstream>

template<typename T>
std::vector<T> parse2vector(std::string& pose_line, size_t num_ele) {
    std::vector<T> vec(num_ele);
    size_t i = 0;
    std::stringstream ssin(pose_line);
    while (ssin.good() && i < num_ele) {
        ssin >> vec[i];
        i++;
    }
    return vec;
}

void avoidJumps(Eigen::Quaterniond &q_cur, Eigen::Quaterniond &q_pre)
{
    // double tmp1 = std::sqrt(pow(q_pre.x() - q_cur.x(), 2) + pow(q_pre.y() - q_cur.y(), 2) + pow(q_pre.z() - q_cur.z(), 2) + pow(q_pre.w() - q_cur.w(), 2));
    // double tmp2 = std::sqrt(pow(q_pre.x() + q_cur.x(), 2) + pow(q_pre.y() + q_cur.y(), 2) + pow(q_pre.z() + q_cur.z(), 2) + pow(q_pre.w() + q_cur.w(), 2));
    // if (tmp1 < tmp2) {
    if (q_cur.w() < 0) {
        q_cur.x() *= -1;
        q_cur.y() *= -1;
        q_cur.z() *= -1;
        q_cur.w() *= -1;
    }
}

void avoidJumps(Eigen::Vector3d &pos_cur, Eigen::Vector3d &pos_pre) {

}


int main(int argc, char** argv) {
    if(argc != 3){
        std::cout << " Wrong number of arguments" << std::endl;
    }

    Eigen::Vector3d pre_pos;
    Eigen::Vector3d cur_pos;        
    Eigen::Vector3d abs_pos; 

    Eigen::Quaterniond pre_q;
    Eigen::Quaterniond cur_q;
    Eigen::Quaterniond abs_q;

    std::vector<double> pose_vec;
    
    size_t i = 0;
    std::ifstream rel_pose_file(argv[1]);
    std::ofstream abs_pose_file(argv[2]);

    std::string cur_poses_line;

    bool is_init_frame = true;
    while (std::getline(rel_pose_file, cur_poses_line)) {
        pose_vec = parse2vector<double>(cur_poses_line, 8);
        if (is_init_frame) {
            is_init_frame = false;
            pre_pos = Eigen::Vector3d {pose_vec[1], pose_vec[2], pose_vec[3]};
            pre_q = Eigen::Quaterniond {pose_vec[4], pose_vec[5], pose_vec[6], pose_vec[7]};
            abs_pose_file << "0 0 0 0 0 0 1\n";
            continue;
        }
        cur_pos = Eigen::Vector3d {pose_vec[1], pose_vec[2], pose_vec[3]};
        cur_q = Eigen::Quaterniond {pose_vec[4], pose_vec[5], pose_vec[6], pose_vec[7]};

        cch::getAbsolutePose(pre_pos, cur_pos, pre_q, cur_q, abs_pos, abs_q);

        // avoidJumps(abs_q, pre_q);

        abs_pose_file << std::to_string(int(pose_vec[0])) << " " 
                      << std::to_string(abs_pos[0]) << " " 
                      << std::to_string(abs_pos[1]) << " " 
                      << std::to_string(abs_pos[2]) << " "
                      << std::to_string(abs_q.x()) << " " 
                      << std::to_string(abs_q.y()) << " " 
                      << std::to_string(abs_q.z()) << " " 
                      << std::to_string(abs_q.w()) << "\n"; 
        pre_pos = std::move(abs_pos);
        pre_q = std::move(abs_q);
    }
    rel_pose_file.close();
    abs_pose_file.close();
    return 0;
}