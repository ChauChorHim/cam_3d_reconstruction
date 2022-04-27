#include "poses_handler.h"
#include "gps_handler.h"
#include <fstream>
#include <iostream>
#include <cmath>

int main() {

    std::string path_to_gps_file {"/home/Autowise/data/autowise/lingang_map5/raw/lingang_map5.csv"};    
    auto gps_handler = cch::GpsHandler(path_to_gps_file);

    // std::string path_to_image_folder {"/home/Autowise/data/autowise/lingang_map5/cam00/cropped_cam00/"};
    std::string path_to_temp_timestamp_file {"/home/Autowise/data/autowise/lingang_map5/cam00/image_files.txt"};

    // auto files_handler = FilesHandler();
    // files_handler.makeFilesList(path_to_image_folder, path_to_temp_timestamp_file, false, false, false, false);

    auto pose_handler = cch::PosesHandler();
    std::ifstream timestamp_file (path_to_temp_timestamp_file);

    std::string cur_timestamp;

    cch::GpsData cur_gps_data;

    Eigen::Vector3d relative_pos; 
    Eigen::Vector3d pre_pos;
    Eigen::Vector3d cur_pos;        

    Eigen::Quaterniond relative_q;
    Eigen::Quaterniond pre_q;
    Eigen::Quaterniond cur_q;

    int i = 0;
    std::ofstream path_to_pose_file ("./poses_gps.txt");
    std::ofstream path_to_translation_file ("./translation_gps.txt");

    /* extract relative pose */
    bool is_init_frame = true;
    while (std::getline(timestamp_file, cur_timestamp)) {
        i++;

        size_t begin = cur_timestamp.find_first_of(' ');
        size_t end = cur_timestamp.find_last_of('.');
        cur_timestamp = cur_timestamp.substr(begin+1, end-begin-1);

        if (is_init_frame) {
            is_init_frame = false;
            cch::GpsData pre_gps_data;
            std::string pre_timestamp = cur_timestamp;
            gps_handler.timestamp2Gps(pre_timestamp, pre_gps_data);
            gps_handler.gps2UtmPose(pre_gps_data, pre_pos, pre_q);
            continue;
        }
        // Get the nearest gps data for current timestamp
        gps_handler.timestamp2Gps(cur_timestamp, cur_gps_data);
        gps_handler.gps2UtmPose(cur_gps_data, cur_pos, cur_q);

        pose_handler.getRelativePose(pre_pos, cur_pos, pre_q, cur_q, relative_pos, relative_q);
        path_to_pose_file << cur_timestamp << " " 
                          << std::to_string(relative_pos[0]) << " " 
                          << std::to_string(relative_pos[1]) << " " 
                          << std::to_string(relative_pos[2]) << " "
                          << std::to_string(relative_q.x()) << " " 
                          << std::to_string(relative_q.y()) << " " 
                          << std::to_string(relative_q.z()) << " " 
                          << std::to_string(relative_q.w()) << "\n"; 
            
        double translation = std::sqrt(std::pow(relative_pos[0], 2) + std::pow(relative_pos[1], 2) + std::pow(relative_pos[2], 2));
        path_to_translation_file << std::to_string(i) << " " << std::to_string(translation) << "\n";

        pre_pos = std::move(cur_pos);
        pre_q = std::move(cur_q);
    }

    path_to_translation_file.close();
    path_to_pose_file.close();

    std::remove(path_to_temp_timestamp_file.c_str());

    return 0;
}