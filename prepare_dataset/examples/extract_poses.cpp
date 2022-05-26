#include "poses_handler.h"
#include "gps_handler.h"
#include <cmath>

int main(int argc, char** argv) {
    if(argc != 5){
        std::cout << " Wrong number of arguments" << std::endl;
    }

    std::string path_to_gps_file(argv[1]);    
    auto gps_handler = cch::GpsHandler(path_to_gps_file);

    std::string path_to_temp_timestamp_file(argv[2]);
    std::ifstream timestamp_file(path_to_temp_timestamp_file);

    std::string cur_timestamp;

    cch::GpsData cur_gps_data;

    Eigen::Vector3d relative_pos; 
    Eigen::Vector3d pre_pos;
    Eigen::Vector3d cur_pos;        
    Eigen::Vector3d init_pos;

    Eigen::Quaterniond relative_q;
    Eigen::Quaterniond pre_q;
    Eigen::Quaterniond cur_q;
    Eigen::Quaterniond init_q;
    
    std::ofstream path_to_pose_file(argv[3]);
    std::ofstream path_to_translation_file(argv[4]);

    bool is_init_frame = true;
    while (std::getline(timestamp_file, cur_timestamp)) {
        size_t begin = cur_timestamp.find_first_of(' ');
        size_t end = cur_timestamp.find_last_of('.');
        cur_timestamp = cur_timestamp.substr(begin+1, end-begin-1);

        std::cout << cur_timestamp << std::endl;

        // Get the nearest gps data for current timestamp
        gps_handler.timestamp2Gps(cur_timestamp, cur_gps_data);
        gps_handler.gps2UtmPose(cur_gps_data, cur_pos, cur_q);

        if (is_init_frame) {
            is_init_frame = false;
            init_pos = cur_pos;
            init_q = cur_q;
            cch::getRelativePose(cur_pos, init_pos, cur_q, init_q, relative_pos, relative_q);
        }

        // Get relative pose from previous frame to current frame
        // cch::getRelativePose(pre_pos, cur_pos, pre_q, cur_q, relative_pos, relative_q);

        // Get absolute pose in the first pose's frame
        cch::getRelativePose(cur_pos, init_pos, cur_q, init_q, relative_pos, relative_q);

        // Transform from gps frame to cam frame
        relative_q = relative_q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());

        path_to_pose_file << cur_timestamp << " " 
                          << std::to_string(relative_pos[0]) << " " 
                          << std::to_string(relative_pos[1]) << " " 
                          << std::to_string(relative_pos[2]) << " "
                          << std::to_string(relative_q.x()) << " " 
                          << std::to_string(relative_q.y()) << " " 
                          << std::to_string(relative_q.z()) << " " 
                          << std::to_string(relative_q.w()) << "\n"; 

        // double translation = std::sqrt(std::pow(relative_pos[0], 2) + std::pow(relative_pos[1], 2) + std::pow(relative_pos[2], 2));
        // path_to_translation_file << std::to_string(i) << " " << std::to_string(translation) << "\n";

        pre_pos = std::move(cur_pos);
        pre_q = std::move(cur_q);
    }

    path_to_translation_file.close();
    path_to_pose_file.close();

    return 0;
}