#include "poses_handler.h"
#include "files_handler.h"
#include "gps_handler.h"
#include <fstream>
#include <iostream>

int main() {

    std::string path_to_gps_file {"/home/Autowise/data/autowise/lingang_map5/raw/lingang_map5.csv"};    
    auto gps_handler = GpsHandler(path_to_gps_file);

    std::string path_to_image_folder {"/home/Autowise/data/autowise/lingang_map5/cam00/raw_cam00/"};
    std::string path_to_temp_timestamp_file {"/home/Autowise/data/autowise/lingang_map5/cam00/temp_timestamps_files.txt"};

    auto files_handler = FilesHandler();
    files_handler.makeFilesList(path_to_image_folder, path_to_temp_timestamp_file, false, false, false, false);

    auto pose_handler = PosesHandler();
    std::ifstream timestamp_file (path_to_temp_timestamp_file);

    std::string cur_timestamp;
    std::string pre_timestamp;

    GpsData pre_gps_data;
    GpsData cur_gps_data;

    Eigen::Vector3d relative_pos; 
    Eigen::Vector3d pre_pos;
    Eigen::Vector3d cur_pos;        

    Eigen::Quaterniond relative_q;
    Eigen::Quaterniond pre_q;
    Eigen::Quaterniond cur_q;

    int i = 0;
    std::ofstream path_to_pose_file ("./poses.txt");

    /* extract relative pose */
    bool is_init_frame = true;
    while (std::getline(timestamp_file, cur_timestamp)) {
        if (is_init_frame) {
            is_init_frame = false;
            pre_timestamp = cur_timestamp;
            gps_handler.timestamp2Gps(pre_timestamp, pre_gps_data);
            gps_handler.gps2UtmPose(pre_gps_data, pre_pos, pre_q);
            i++;
            continue;
        }
        // Get the nearest gps data for current timestamp
        gps_handler.timestamp2Gps(cur_timestamp, cur_gps_data);
        gps_handler.gps2UtmPose(cur_gps_data, cur_pos, cur_q);

        pose_handler.getRelativePose(pre_pos, cur_pos, pre_q, cur_q, relative_pos, relative_q);
        path_to_pose_file << cur_timestamp << " " 
                          << std::to_string(relative_q.x()) << " " 
                          << std::to_string(relative_q.y()) << " " 
                          << std::to_string(relative_q.z()) << " " 
                          << std::to_string(relative_q.w()) << " " 
                          << std::to_string(relative_pos[0]) << " " 
                          << std::to_string(relative_pos[1]) << " " 
                          << std::to_string(relative_pos[2]) << "\n";

        pre_timestamp = cur_timestamp;
        pre_gps_data = std::move(cur_gps_data);
        pre_pos = std::move(cur_pos);
        pre_q = std::move(cur_q);
    }


    path_to_pose_file.close();

    std::remove(path_to_temp_timestamp_file.c_str());

    return 0;
}