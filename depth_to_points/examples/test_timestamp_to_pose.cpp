#include "timestamp_to_pose.h"
#include <iostream>
#include <fstream>

int main () {
    std::vector<std::string> path_to_gps_files;
    path_to_gps_files.push_back("/home/Autowise/data/autowise/lingang_map5/raw/_2022-02-21-15-04-38.csv");
    path_to_gps_files.push_back("/home/Autowise/data/autowise/lingang_map5/raw/_2022-02-21-15-22-41.csv");
    Gps2Pose gps2pose = Gps2Pose();

    for (auto path_to_gps_file : path_to_gps_files) {
        gps2pose.addGpsData(path_to_gps_file);
    }

    std::string path_to_list {"/home/Autowise/manydepth/splits/lingang_map5_cam00/image_files.txt"};
    std::ifstream list_file(path_to_list);
    std::vector<std::string> timestamp_list;
    std::string line;
    while (std::getline(list_file, line)) {
        timestamp_list.push_back(line.substr(line.find(' ')+1, line.find_last_of('.')-2));
    }

    std::string timestamp;
    Eigen::Vector3d pos;
    Eigen::Quaterniond q_inv;

    gps2pose.initializePose();

    for (size_t i = 0; i < timestamp_list.size(); ++i) {
        timestamp = timestamp_list[i];
        gps2pose.timestamp2Pose(timestamp, pos, q_inv);
    }
    
    return 0;
}