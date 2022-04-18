#include <iostream>

#include "depth_to_points.h"
#include "npy.h"

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " fx fy cx cy path_to_npy path_to_rgb path_to_output_pcd\n"; 
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {
    if(argc != 8){
        std::cout << " Wrong number of arguments" << std::endl;
        printHelp(argv[0]);
    }

    float fx = (float)strtod(argv[1], nullptr);
    float fy = (float)strtod(argv[2], nullptr);
    float cx = (float)strtod(argv[3], nullptr);
    float cy = (float)strtod(argv[4], nullptr);
    PointCloudSaver point_cloud_saver = PointCloudSaver(fx, fy, cx, cy);

    std::string path_to_depth_npy {argv[5]};
    std::string path_to_image {argv[6]};
    std::string path_to_pcd_file {argv[7]};

    std::cout << path_to_depth_npy << "\n" << path_to_image << "\n";

    // point_cloud_saver.saveOnePointCloud(path_to_depth_npy, path_to_image, path_to_pcd_file);
    point_cloud_saver.saveOnePointCloud(path_to_depth_npy, path_to_image, path_to_pcd_file, true);

    return 0;
}
