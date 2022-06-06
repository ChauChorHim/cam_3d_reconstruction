#include "poses_handler.h"
#include "gps_handler.h"
#include <cmath>
#include <iostream>
#include <fstream>

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

    Eigen::Vector3d rel_pos; 
    Eigen::Vector3d abs_pos; 
    Eigen::Vector3d pre_pos;
    Eigen::Vector3d cur_pos;        
    Eigen::Vector3d init_pos;

    Eigen::Quaterniond rel_q;
    Eigen::Quaterniond abs_q;
    Eigen::Quaterniond pre_q;
    Eigen::Quaterniond cur_q;
    Eigen::Quaterniond init_q;
    
    // std::ofstream path_to_pose_file(argv[3]);
    std::ofstream path_to_translation_file(argv[4]);

    bool is_init_frame_absolute = true;
    bool is_init_frame_relative = true;
    while (std::getline(timestamp_file, cur_timestamp)) {
        size_t begin = cur_timestamp.find_first_of(' ');
        size_t end = cur_timestamp.find_last_of('.');
        cur_timestamp = cur_timestamp.substr(begin+1, end-begin-1);

        std::cout << cur_timestamp << std::endl;

        // Get the nearest gps data for current timestamp
        gps_handler.timestamp2Gps(cur_timestamp, cur_gps_data);

        // gps data to utm pose
        gps_handler.gps2UtmPose(cur_gps_data, cur_pos, cur_q);

        if (is_init_frame_absolute) {
            is_init_frame_absolute = false;
            init_pos = cur_pos;
            init_q = cur_q;
        }

        /* Get absolute pose at the first pose's frame */
        cch::getRelativePose(cur_pos, init_pos, cur_q, init_q, abs_pos, abs_q);

        // Transform from gps frame to cam frame
        abs_q = abs_q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());

        // Write to the file
        // path_to_pose_file << cur_timestamp << " " 
        //                   << std::to_string(abs_pos[0]) << " " 
        //                   << std::to_string(abs_pos[1]) << " " 
        //                   << std::to_string(abs_pos[2]) << " "
        //                   << std::to_string(abs_q.x()) << " " 
        //                   << std::to_string(abs_q.y()) << " " 
        //                   << std::to_string(abs_q.z()) << " " 
        //                   << std::to_string(abs_q.w()) << "\n";

        /* ********************************************************** */

        /* Get relative pose from previous frame to current frame */
        if (is_init_frame_relative) {
            is_init_frame_relative = false;
            pre_pos = abs_pos;
            pre_q = abs_q;
        }
        cch::getRelativePose(pre_pos, abs_pos, pre_q, abs_q, rel_pos, rel_q);

        // Write to the file
        // path_to_pose_file << cur_timestamp << " " 
        //                   << std::to_string(rel_pos[0]) << " " 
        //                   << std::to_string(rel_pos[1]) << " " 
        //                   << std::to_string(rel_pos[2]) << " "
        //                   << std::to_string(rel_q.x()) << " " 
        //                   << std::to_string(rel_q.y()) << " " 
        //                   << std::to_string(rel_q.z()) << " " 
        //                   << std::to_string(rel_q.w()) << "\n"; 

        double translation = std::sqrt(std::pow(rel_pos[0], 2) + std::pow(rel_pos[1], 2) + std::pow(rel_pos[2], 2));
        path_to_translation_file << cur_timestamp << " " << std::to_string(translation) << "\n";

        pre_pos = std::move(abs_pos);
        pre_q = std::move(abs_q);
    }

    path_to_translation_file.close();
    // path_to_pose_file.close();

    return 0;
}