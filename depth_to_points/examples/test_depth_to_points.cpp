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
    BackProjection backProjection = BackProjection(height, width);
    Eigen::Matrix3d K;
    K << 547.92, 0, 331.33,                                       // *** 
         0, 628.01, 161.58,
         0, 0, 1.0;
    PCDSaver pcdSaver = PCDSaver(height, width, K);

    std::vector<std::string> path_to_gps_files;
    path_to_gps_files.push_back("assets/_2022-02-21-15-04-38.csv");          // *** 
    path_to_gps_files.push_back("assets/_2022-02-21-15-22-41.csv");          // *** 
    path_to_gps_files.push_back("assets/_2022-02-21-15-33-24.csv");          // *** 

    std::string path_to_image_list {"assets/image_files.txt"};                // ***
    std::string path_to_depth_list {"assets/depth_files.txt"};                // ***

    std::string path_to_pcd_file = "./scene_lingang_map4.pcd";                            // ***


    /* --- Load Raw Data --- */
    // get all gps message data
    Gps2Pose gps2pose = Gps2Pose();
    for (auto path_to_gps_file : path_to_gps_files) {
        gps2pose.addGpsData(path_to_gps_file);
    }
    // set the first gps message as initial state
    gps2pose.initializePose(nullptr);


    std::ifstream image_list_file(path_to_image_list);
    std::ifstream depth_list_file(path_to_depth_list);
    std::ifstream timestamp_list_file(path_to_image_list);

    std::vector<std::string> image_list;
    std::vector<std::string> depth_list;
    std::vector<std::string> timestamp_list;

    std::string line;
    // push back timestamp
    while (std::getline(timestamp_list_file, line)) {
        timestamp_list.push_back(line.substr(line.find_last_of('/')+1, line.find_last_of('.')-2));              // ***
    }

    std::cout << " --- \n" << timestamp_list.size() << " image/depth with the same timestamp --- \n\n";

    // push back image list
    while (std::getline(image_list_file, line)) {
        image_list.push_back(line);
    }

    // push back depth list
    while (std::getline(depth_list_file, line)) {
        depth_list.push_back(line);
    }

    std::vector<Eigen::Vector3d> pos_list;
    // pos_list.reserve(timestamp_list.size());
    std::vector<Eigen::Quaterniond> q_list;
    // q_inv_list.reserve(timestamp_list.size());

    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    // store pos and q_inv
    for (size_t i = 0; i < timestamp_list.size(); ++i) {
        gps2pose.timestamp2Pose(timestamp_list[i], pos, q);
        // pos_list[i] = pos;
        // q_inv_list[i] = q_inv;
        pos_list.push_back(pos);
        q_list.push_back(q);
    }

    /* --- Main Loop --- */
    for (size_t idx = 0; idx < pos_list.size(); idx = idx + 1) {
        std::cout << "Current idx: " << idx << "\n";

        std::string cur_path_to_image = image_list[idx];
        std::string cur_path_to_depth = depth_list[idx];
        Eigen::Vector3d cur_pos = pos_list[idx];
        Eigen::Quaterniond cur_q = q_list[idx];

        pcdSaver.addDepthMap(cur_path_to_depth, cur_path_to_image,
                                cur_pos, cur_q);

        if (idx % 20 == 1) {
            std::string save_dir {"./scene_lingang_map_"};
            save_dir = save_dir + std::to_string(idx) + ".pcd";
            pcdSaver.save(save_dir);
        }
    }

    pcdSaver.save(path_to_pcd_file);
    
    return 0;
}