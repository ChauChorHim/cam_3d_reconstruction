#include "poses_handler.h"
#include "files_handler.h"
#include <fstream>
#include <iostream>

int main() {
    auto pose_handler = PosesHandler();
    std::string path_to_gps_file {"/home/Autowise/zcq_tools/cam-3d-reconstruction/depth_to_points/examples/assets/lingang_map5.csv"};    
    pose_handler.addGpsData(path_to_gps_file);

    std::string path_to_image_folder {"/home/Autowise/data/autowise/lingang_map5/cam00/unrectified_cam00/"};
    std::string path_to_temp_timestamp_file {"/home/Autowise/data/autowise/lingang_map5/cam00/temp_timestamps_files.txt"};

    auto files_handler = FilesHandler();
    files_handler.makeFilesList(path_to_image_folder, path_to_temp_timestamp_file, false, false, false, false);

    std::ifstream timestamp_file (path_to_temp_timestamp_file);
    std::string cur_timestamp;
    Eigen::Vector3d pos; 
    Eigen::Quaterniond q;

    int i = 0;
    std::ofstream path_to_pose_file ("./poses.txt");
    /* get absolute pose */
    // while(std::getline(timestamp_file, cur_timestamp)) {
    //     pose_handler.getAbsolutePose(cur_timestamp, pos, q);
    //     path_to_pose_file << std::to_string(q.x()) << " " << std::to_string(q.y()) << " " << std::to_string(q.z()) << " " << std::to_string(q.w()) << " " << std::to_string(pos[0]) << " " << std::to_string(pos[1]) << " " << std::to_string(pos[2]) << "\n";
    //     // path_to_pose_file << std::to_string(pos[0]) << " " << std::to_string(pos[1]) << " " << std::to_string(pos[2]) << "\n";
    // }

    /* get relative pose */
    std::string pre_timestamp;
    bool is_init_frame = true;
    while (std::getline(timestamp_file, cur_timestamp)) {
        if (is_init_frame) {
            is_init_frame = false;
            pre_timestamp = cur_timestamp;
            continue;
        }
        pose_handler.getRelativePose(pre_timestamp, cur_timestamp, pos, q);
        path_to_pose_file << std::to_string(i++) << " " << std::to_string(q.x()) << " " << std::to_string(q.y()) << " " << std::to_string(q.z()) << " " << std::to_string(q.w()) << " " << std::to_string(pos[0]) << " " << std::to_string(pos[1]) << " " << std::to_string(pos[2]) << "\n";
        pre_timestamp = cur_timestamp;
    }


    path_to_pose_file.close();

    std::remove(path_to_temp_timestamp_file.c_str());

    return 0;
}