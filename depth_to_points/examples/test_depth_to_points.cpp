#include "timestamp_to_pose.h"
#include "depth_to_points.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <set>
#include <filesystem>

struct AllData {
    std::string timestamp;
    std::string path_to_image;
    std::string path_to_depth;
    Eigen::Vector3d pos;
    Eigen::Quaterniond q_inv;

};

int main () {
    /* Please modify all stuffs in front of '// ***'

    /* --- Config Settup --- */
    int height = 320;                                                        // ***
    int width = 640;                                                         // ***
    double min_depth = 0.1;                                                  // *** 
    double max_depth = 100.0;                                                // ***
    BackProjection backProjection = BackProjection(height, width);
    Eigen::Matrix3d K;
    K << 0.68 * width, 0, 1.0 * width,                                       // *** 
         0, 1.36 * height, 1.23 * height,
         0, 0, 1.0;
    Eigen::Matrix3d K_inv = K.inverse();

    PCDSaver pcdSaver = PCDSaver(height, width, min_depth, max_depth, K);

    std::vector<std::string> path_to_gps_files;
    path_to_gps_files.push_back("assets/_2022-02-21-15-04-38.csv");          // *** 
    path_to_gps_files.push_back("assets/_2022-02-21-15-22-41.csv");          // ***

    std::string path_to_image_list {"assets/image_list.txt"};                // ***
    std::string path_to_depth_list {"assets/depth_list.txt"};                // ***

    std::string path_to_pcd_file = "./scene.pcd";                            // ***


    /* --- Load Raw Data --- */
    // get all gps message data
    Gps2Pose gps2pose = Gps2Pose();
    for (auto path_to_gps_file : path_to_gps_files) {
        gps2pose.addGpsData(path_to_gps_file);
    }
    // set the first gps message as initial state
    gps2pose.initializePose();


    std::ifstream image_list_file(path_to_image_list);
    std::ifstream depth_list_file(path_to_depth_list);
    std::ifstream timestamp_list_file(path_to_image_list);

    std::vector<std::string> timestamp_list;
    std::vector<std::string> depth_list;
    std::vector<std::string> image_list;

    std::string line;
    // push back timestamp
    while (std::getline(timestamp_list_file, line)) {
        timestamp_list.push_back(line.substr(line.find_last_of('/')+1, line.find_last_of('.')-2));              // ***
    }

    std::cout << "\nWe get " << timestamp_list.size() << " image/depth with the same timestamp\n\n";

    // push back image list
    while (std::getline(image_list_file, line)) {
        image_list.push_back(line);
    }

    // push back depth list
    while (std::getline(depth_list_file, line)) {
        depth_list.push_back(line);
    }

    std::vector<Eigen::Vector3d> pos_list;
    pos_list.reserve(timestamp_list.size());
    std::vector<Eigen::Quaterniond> q_inv_list;
    q_inv_list.reserve(timestamp_list.size());
    
    Eigen::Vector3d pos;
    Eigen::Quaterniond q_inv;
    // store pos and q_inv
    for (size_t i = 0; i < timestamp_list.size(); ++i) {
        gps2pose.timestamp2Pose(timestamp_list[i], pos, q_inv);
        pos_list[i] = pos;
        q_inv_list[i] = q_inv;
    }

    
    /* --- Main Loop --- */
    for (size_t idx = 0; idx < timestamp_list.size(); ++idx) {

        std::string cur_path_to_image = image_list[idx];
        std::string cur_path_to_depth = depth_list[idx];
        Eigen::Vector3d cur_pos = pos_list[idx];
        Eigen::Quaterniond cur_q_inv = q_inv_list[idx];

        pcdSaver.addDepthMap(cur_path_to_depth, cur_path_to_image,
                                cur_pos, cur_q_inv);
    }

    pcdSaver.save(path_to_pcd_file);
    
    return 0;
}