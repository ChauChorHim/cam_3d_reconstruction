#include <iostream>
#include <filesystem>

#include "depth_to_points.h"
#include "files_handler.h"
#include "npy.h"

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " fx fy cx cy dir_to_npy dir_to_rgb dir_to_output_pcd\n"; 
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
    cch::PointCloudSaver point_cloud_saver = cch::PointCloudSaver(fx, fy, cx, cy);

    std::string dir_to_depth_npy {argv[5]};
    std::string dir_to_image {argv[6]};
    std::string dir_to_mask {argv[7]};
    std::string dir_to_pcd_file {argv[8]};

    cch::validateFolderDir(dir_to_depth_npy);
    cch::validateFolderDir(dir_to_image);
    cch::validateFolderDir(dir_to_mask);
    cch::validateFolderDir(dir_to_pcd_file);

    std::cout << "\nDirectory to depth npy: " << dir_to_depth_npy 
              << "\nDirectory to image: " << dir_to_image 
              << "\nDirectory to mask: " << dir_to_mask
              << "\nDirectory to output pcd: " <<  dir_to_pcd_file;

    /* --- Load Raw Data --- */
    
    std::set<std::filesystem::path> depth_list_set;
    std::set<std::filesystem::path> mask_list_set;
    std::set<std::filesystem::path> image_list_set;

    for (const auto &path_to_file: std::filesystem::directory_iterator(dir_to_depth_npy))
        depth_list_set.insert(path_to_file.path());
    for (const auto &path_to_file: std::filesystem::directory_iterator(dir_to_mask))
        mask_list_set.insert(path_to_file.path());
    for (const auto &path_to_file: std::filesystem::directory_iterator(dir_to_image))
        image_list_set.insert(path_to_file.path());

    assert (depth_list_set.size() == image_list_set.size());

    std::vector<std::filesystem::path> depth_list (depth_list_set.begin(), depth_list_set.end());
    std::vector<std::filesystem::path> mask_list (mask_list_set.begin(), mask_list_set.end());
    std::vector<std::filesystem::path> image_list (image_list_set.begin(), image_list_set.end());

    std::cout << " --- \n" << image_list.size() << " image/depth pairs are in the buffer --- \n\n";

    for (size_t i = 0; i < depth_list.size(); ++i) {
        std::string path_to_depth = depth_list[i];
        std::string path_to_mask = mask_list[i];
        std::string path_to_image = image_list[i];
        point_cloud_saver.depthToPointCloud(path_to_depth, path_to_mask, &path_to_image);

        std::string path_to_pcd = (std::filesystem::path(dir_to_pcd_file) / (image_list[i]).stem()).c_str();
        path_to_pcd += ".pcd";

        point_cloud_saver.downSample();
        // point_cloud_saver.removeOutlier();

        point_cloud_saver.save(path_to_pcd);
        point_cloud_saver.clear();
    }

    return 0;
}
