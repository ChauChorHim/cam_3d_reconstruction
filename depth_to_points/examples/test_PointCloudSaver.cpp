#include <iostream>

#include "depth_to_points.h"
#include "npy.h"

int main() {
    float fx = 290.18;
    float fy = 405.85;
    float cx = 306.42;
    float cy = 177.89;
    // float fx = 547.92;
    // float fy = 628.01;
    // float cx = 331.33;
    // float cy = 161.58;
    PointCloudSaver point_cloud_saver = PointCloudSaver(fx, fy, cx, cy);

    std::string test_path_to_depth_npy = "/home/Autowise/zcq_tools/depth_to_points/examples/1645427078.200816_disp_multi.npy";
    std::string test_path_to_image = "/home/Autowise/zcq_tools/depth_to_points/examples/1645427078.200816_disp_multi.jpeg";

    point_cloud_saver.addPointCloud(test_path_to_depth_npy, test_path_to_image);
    return 0;
}