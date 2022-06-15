#include <filesystem>

#include "pc_tools.h"
#include "files_handler.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

void line2PoseAndQ(std::string& line, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    std::istringstream line_stream(line);
    double ele;
    std::vector<double> line_vec;
    for (int i = 1; i < 9; ++i) {
        line_stream >> ele;
        line_vec.push_back(ele);
    }
    pos = Eigen::Vector3d(line_vec[1], line_vec[2], line_vec[3]);
    q = Eigen::Quaterniond(line_vec[7], line_vec[4], line_vec[5], line_vec[6]);
}

int main(int argc, char** argv) {

    if(argc != 8){
        std::cout << " Wrong number of arguments" << std::endl;
    }

    /* --- Config Settup --- */

    float fx = (float)strtod(argv[1], nullptr);
    float fy = (float)strtod(argv[2], nullptr);
    float cx = (float)strtod(argv[3], nullptr);
    float cy = (float)strtod(argv[4], nullptr);
    cch::PointCloudSaver point_cloud_saver = cch::PointCloudSaver(fx, fy, cx, cy);

    std::string dir_to_pcd {argv[5]};
    std::string path_to_output_pcd {argv[6]};
    std::string path_to_pose {argv[7]};
    
    cch::validateFolderDir(dir_to_pcd);

    /* --- Load Raw Data --- */
    std::set<std::filesystem::path> pcd_list_set;

    for (const auto &path_to_file: std::filesystem::directory_iterator(dir_to_pcd))
        pcd_list_set.insert(path_to_file.path());

    std::vector<std::filesystem::path> pcd_list (next(pcd_list_set.begin()), pcd_list_set.end());

    std::cout << " --- \n" << pcd_list.size() << " pcd files are in the buffer --- \n\n";

    // Use pose from a txt file
    std::fstream pose_file;
    pose_file.open(path_to_pose, std::ios::in);
    assert(pose_file.is_open());
    std::vector<std::string> pose_lines;
    std::string cur_pose_line;

    while (std::getline(pose_file, cur_pose_line)) {
        pose_lines.push_back(cur_pose_line);
    }

    pose_file.close();

    /* --- Main Loop --- */
    // Some temp parameters
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    pcl::PointCloud<pcl::PointXYZRGB> cur_pc_cam;

    for (size_t idx = 300; idx < pcd_list.size(); idx++) {
        if (idx % 2 == 1) {
        // if (true) {
            std::cout << "\nCurrent pcd idx: " << idx << "\n";
            cur_pc_cam.clear();
            pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_list[idx], cur_pc_cam);
            line2PoseAndQ(pose_lines[idx], pos, q);
            PointCloudT cur_pc_world;
            pcl::transformPointCloud(cur_pc_cam, cur_pc_world, pos, q);
            point_cloud_saver.addPointCloud(cur_pc_world);
        }

        if (idx >= 600)
            break;
    }

    /* --- Filter out outlier and downsample pointcloud --- */

    point_cloud_saver.removeOutlier();

    point_cloud_saver.save(path_to_output_pcd);

    return 0;
}
