#include "pc_tools.h"

#include <vector>
#include <cstring>
#include <set>
#include <filesystem>
#include <memory>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

void showMatrix(Eigen::Matrix4f mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << mat(i, j) << " ";
        }
        std::cout << "\n";
    }
}

void saveTransMatrix(std::string& path_to_output_pose,
                     std::vector<Eigen::Matrix4f>& poses_output_vec, 
                     std::vector<std::string>& timestamps) {
    std::fstream pose_file;
    pose_file.open(path_to_output_pose, std::ios::out);
    assert(pose_file.is_open());

    std::string cur_pose_line;

    for (int i = 0; i < poses_output_vec.size(); ++i) {
        Eigen::Matrix4f cur_pose = poses_output_vec[i];
        Eigen::Matrix3f rot_mat = cur_pose.block<3, 3>(0, 0);
        Eigen::Vector3f t = cur_pose.block<3, 1>(0, 3);
        Eigen::Quaternionf q(rot_mat);

        cur_pose_line = timestamps[i] + " " + std::to_string(t[0]) + " " + std::to_string(t[1]) + " " + std::to_string(t[2]) + " "
                        + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z()) + " " + std::to_string(q.w());
        pose_file << cur_pose_line << "\n";
    }

    pose_file.close();
}

bool loadPoses(std::string& path_to_pose,
               std::vector<Eigen::Matrix4f>& poses_input_vec,
               std::vector<std::string>& timestamps) {
    poses_input_vec.clear();

    std::fstream pose_file;
    pose_file.open(path_to_pose, std::ios::in);
    assert(pose_file.is_open());
    std::string cur_pose_line;

    while (std::getline(pose_file, cur_pose_line)) {
        std::istringstream line_stream(cur_pose_line);
        double ele;
        std::vector<double> line_vec;
        for (int i = 0; i < 9; ++i) {
            line_stream >> ele;
            line_vec.push_back(ele);
        }
        auto pos = Eigen::Vector3f(line_vec[1], line_vec[2], line_vec[3]);
        auto q = Eigen::Quaternionf(line_vec[7], line_vec[4], line_vec[5], line_vec[6]);
        timestamps.push_back(std::to_string(line_vec[0]));

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3, 3>(0, 0) = q.toRotationMatrix();
        trans.block<3, 1>(0, 3) = pos;
        poses_input_vec.push_back(trans);
    }

    pose_file.close();
    return true;
}

void icpOptimizePose(std::vector<std::string>& pc_vec, 
                     std::vector<Eigen::Matrix4f>& poses_input_vec, 
                     std::vector<Eigen::Matrix4f>& poses_output_vec) { 

    pcl::PCDReader reader;
    // pcl::PCDWriter writer;

    std::string path_cloud = pc_vec[0];
    PointCloudT::Ptr ptr_cloud_in (new PointCloudT);
    reader.read(path_cloud, *ptr_cloud_in);
    pcl::transformPointCloud(*ptr_cloud_in, *ptr_cloud_in, poses_input_vec[0]);

    poses_output_vec.push_back(poses_input_vec[0]);

    // for (int i = 1; i < pc_vec.size(); ++i) {
    for (int i = 1; i < 101; ++i) {
        std::cout << i << " ";
        path_cloud = pc_vec[i];
        PointCloudT::Ptr ptr_cloud_out (new PointCloudT);
        reader.read(path_cloud, *ptr_cloud_out);
        pcl::transformPointCloud(*ptr_cloud_out, *ptr_cloud_out, poses_input_vec[i]);

        // showMatrix(poses_input_vec[i]);

        // writer.write("tmp/" + std::to_string(i) + "_in.pcd", *ptr_cloud_in);
        // writer.write("tmp/" + std::to_string(i) + "_out.pcd", *ptr_cloud_out);

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(ptr_cloud_in);
        icp.setInputTarget(ptr_cloud_out);
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (0.05);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (100);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1);

        PointCloudT cloud_in_trans;
        Eigen::Matrix4f init_guess = poses_input_vec[i].inverse() * poses_input_vec[i-1];
        icp.align(cloud_in_trans, init_guess);

        if (icp.hasConverged()) 
            std::cout << "ICP converged. The score is " << icp.getFitnessScore() << "\n";
        else 
            std::cout << "ICP did not converge." << std::endl;

        Eigen::Matrix4f pose_output = icp.getFinalTransformation();
        poses_output_vec.push_back(pose_output * poses_output_vec.back());
        // poses_output_vec.push_back(pose_output);

        ptr_cloud_in = ptr_cloud_out;
    }

}

int main(int argc, char** argv) {
    std::string pc_folder {"/home/Autowise/data/autowise/lingang_map5/depth/depth_03/pcd/"};
    std::set<std::string> pc_list_set;
    if (!cch::loadPcdPath(pc_folder, pc_list_set)) {
        return 1;
    }
    std::vector<std::string> pc_vec (pc_list_set.begin(), pc_list_set.end());

    std::string path_to_input_pose {"/home/Autowise/data/autowise/lingang_map5/poses/gps/poses_cam.txt"};
    std::vector<Eigen::Matrix4f> poses_input_vec;
    std::vector<std::string> timestamps;
    if (!loadPoses(path_to_input_pose, poses_input_vec, timestamps)) {
        return 1;
    }

    std::vector<Eigen::Matrix4f> poses_output_vec;
    icpOptimizePose(pc_vec, poses_input_vec, poses_output_vec);

    std::string path_to_output_pose {"/home/Autowise/data/autowise/lingang_map5/poses/opt/poses_cam_icp.txt"};
    saveTransMatrix(path_to_output_pose, poses_output_vec, timestamps);

    return 0;
}