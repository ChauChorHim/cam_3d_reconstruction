#include "depth_to_points.h"
#include "files_handler.h"
#include "rotation_handler.h"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " fx fy cx cy path_to_npy path_to_rgb path_to_output_pcd path_to_pose\n"; 
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {

    if(argc != 9){
        std::cout << " Wrong number of arguments" << std::endl;
        printHelp(argv[0]);
    }

    /* --- Config Settup --- */

    float fx = (float)strtod(argv[1], nullptr);
    float fy = (float)strtod(argv[2], nullptr);
    float cx = (float)strtod(argv[3], nullptr);
    float cy = (float)strtod(argv[4], nullptr);

    std::string path_to_depth_folder {argv[5]};
    std::string path_to_image_folder {argv[6]};
    std::string path_to_output_pcd {argv[7]};
    std::string path_to_pose {argv[8]};

    std::string path_to_depth_list = "./depth_files.txt";
    cch::makeFilesList(path_to_depth_folder, path_to_depth_list, false, false, true);

    std::string path_to_image_list = "./image_files.txt";
    cch::makeFilesList(path_to_image_folder, path_to_image_list, false, false, true);

    /* --- Load Raw Data --- */

    std::string line;
    // push back image list
    std::ifstream image_list_file(path_to_image_list);
    std::vector<std::string> image_list;
    while (std::getline(image_list_file, line)) {
        image_list.push_back(line);
    }

    // push back depth list
    std::ifstream depth_list_file(path_to_depth_list);
    std::vector<std::string> depth_list;
    while (std::getline(depth_list_file, line)) {
        depth_list.push_back(line);
    }

    // make sure size of image and depth are the same
    assert(image_list.size() == depth_list.size());
    std::cout << " --- \n" << image_list.size() << " image/depth pairs are in the buffer --- \n\n";

    std::cout << "\n";

    // Use pose from a txt file
    std::fstream pose_file;
    pose_file.open(path_to_pose, std::ios::in);

    assert(pose_file.is_open());

    /* --- Main Loop --- */

    cch::PointCloudSaver point_cloud_saver = cch::PointCloudSaver(fx, fy, cx, cy);

    // Some temp parameters
    std::string cur_pose_line;
    Eigen::Vector3d relative_pos, absolute_pos(Eigen::Vector3d::Zero());
    Eigen::Quaterniond relative_q, absolute_q(Eigen::Quaternion(0, 0, 0, 1));

    for (size_t idx = 0; idx < image_list.size(); idx++) {
        std::string cur_path_to_image = image_list[idx];
        std::string cur_path_to_depth = depth_list[idx];

        cch::readline(pose_file, idx, cur_pose_line);
        cch::line2PoseAndQ(cur_pose_line, relative_pos, relative_q);

        absolute_pos = absolute_pos + relative_pos;
        // absolute_q = relative_q * absolute_q;
        absolute_q =  absolute_q * relative_q;

        // std::cout << absolute_pos[0] << " " << absolute_pos[1] << " " << absolute_pos[2] << "\n";

        if (idx % 30 == 1) {
            std::cout << "\nCurrent image/depth idx: " << idx << "\n";
            point_cloud_saver.addPointCloud(cur_path_to_depth, cur_path_to_image, absolute_pos, absolute_q);

            if (idx >= 0)
                break;
        }
    }

    point_cloud_saver.save(path_to_output_pcd);

    pose_file.close();
    std::remove(path_to_image_list.c_str());
    std::remove(path_to_image_list.c_str());
    return 0;
}