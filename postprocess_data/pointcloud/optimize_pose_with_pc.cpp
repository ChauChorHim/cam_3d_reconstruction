#include "pc_tools.h"

#include <vector>
#include <cstring>
#include <set>
#include <filesystem>
#include <memory>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
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
                     std::vector<Eigen::Isometry3f>& poses_output_vec, 
                     std::vector<std::string>& timestamps) {
    std::fstream pose_file;
    pose_file.open(path_to_output_pose, std::ios::out);
    assert(pose_file.is_open());

    std::string cur_pose_line;

    for (int i = 0; i < poses_output_vec.size(); ++i) {
        Eigen::Isometry3f cur_pose = poses_output_vec[i];
        Eigen::Matrix3f rot_mat = cur_pose.rotation();
        Eigen::Vector3f t = cur_pose.translation();
        Eigen::Quaternionf q(rot_mat);

        cur_pose_line = timestamps[i] + " " + std::to_string(t[0]) + " " + std::to_string(t[1]) + " " + std::to_string(t[2]) + " "
                        + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z()) + " " + std::to_string(q.w());
        pose_file << cur_pose_line << "\n";
    }

    pose_file.close();
}

void loadPoses(std::string& path_to_pose,
               std::vector<Eigen::Isometry3f>& poses_input_vec,
               std::vector<std::string>& timestamps) {
    poses_input_vec.clear();

    std::fstream pose_file;
    pose_file.open(path_to_pose, std::ios::in);
    assert(pose_file.is_open());
    std::string cur_pose_line;

    while (std::getline(pose_file, cur_pose_line)) {
        std::istringstream line_stream(cur_pose_line);
        std::string cur_timestamp;
        line_stream >> cur_timestamp;
        timestamps.push_back(cur_timestamp);

        float ele;
        std::vector<double> line_vec;
        for (int i = 0; i < 7; ++i) {
            line_stream >> ele;
            line_vec.push_back(ele);
        }
        Eigen::Vector3f pos = Eigen::Vector3f(line_vec[0], line_vec[1], line_vec[2]);
        Eigen::Quaternionf q = Eigen::Quaternionf(line_vec[6], line_vec[3], line_vec[4], line_vec[5]);

        Eigen::Isometry3f T(q);
        T.pretranslate(pos);
        poses_input_vec.push_back(T);
    }
    pose_file.close();
}

void ndtOptimizePose(std::vector<std::string>& pc_vec, 
                     std::vector<Eigen::Isometry3f>& poses_input_vec, 
                     std::vector<Eigen::Isometry3f>& poses_output_vec) { 

    pcl::PCDReader reader;
    // pcl::PCDWriter writer;

    std::string path_cloud = pc_vec[0];
    PointCloudT::Ptr ptr_cloud_in (new PointCloudT);
    reader.read(path_cloud, *ptr_cloud_in);
    pcl::transformPointCloud(*ptr_cloud_in, *ptr_cloud_in, poses_input_vec[0].matrix());

    poses_output_vec.push_back(poses_input_vec[0]);

    for (int i = 1; i < pc_vec.size(); ++i) {
    // for (int i = 1; i < 201; ++i) {
        std::cout << i << " ";
        path_cloud = pc_vec[i];
        PointCloudT::Ptr ptr_cloud_out (new PointCloudT);
        reader.read(path_cloud, *ptr_cloud_out);
        pcl::transformPointCloud(*ptr_cloud_out, *ptr_cloud_out, poses_input_vec[i].matrix());

        pcl::NormalDistributionsTransform<PointT, PointT> ndt;
        ndt.setInputSource(ptr_cloud_in);
        ndt.setInputTarget(ptr_cloud_out);

        ndt.setStepSize (0.1);
        ndt.setResolution (1.0);
        // Set the maximum number of iterations (criterion 1)
        ndt.setMaximumIterations (35);
        // Set the transformation epsilon (criterion 2)
        ndt.setTransformationEpsilon (0.01);
        

        PointCloudT cloud_in_trans;
        // Eigen::Isometry3f init_guess = poses_input_vec[i].inverse() * poses_input_vec[i-1];
        Eigen::Isometry3f init_guess = poses_input_vec[i-1].inverse() * poses_input_vec[i];
        ndt.align(cloud_in_trans, init_guess.matrix());

        if (ndt.hasConverged()) 
            std::cout << "converged. The score is " << ndt.getFitnessScore() << "\n";
        else 
            std::cout << "did not converge." << std::endl;

        Eigen::Matrix4f matrix_output = ndt.getFinalTransformation();
        Eigen::Isometry3f pose_output;
        pose_output.matrix() = matrix_output;
        // poses_output_vec.push_back(pose_output * poses_output_vec.back());
        poses_output_vec.push_back(poses_output_vec.back() * pose_output);

        ptr_cloud_in = ptr_cloud_out;
    }

}

void icpOptimizePose(std::vector<std::string>& pc_vec, 
                     std::vector<Eigen::Isometry3f>& poses_input_vec, 
                     std::vector<Eigen::Isometry3f>& poses_output_vec) { 

    pcl::PCDReader reader;
    // pcl::PCDWriter writer;

    std::string path_cloud = pc_vec[0];
    PointCloudT::Ptr ptr_cloud_in (new PointCloudT);
    reader.read(path_cloud, *ptr_cloud_in);
    pcl::transformPointCloud(*ptr_cloud_in, *ptr_cloud_in, poses_input_vec[0].matrix());

    poses_output_vec.push_back(poses_input_vec[0]);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance (0.03);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (20);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    for (int i = 1; i < pc_vec.size(); ++i) {
    // for (int i = 1; i < 101; ++i) {
        std::cout << i << " ";
        path_cloud = pc_vec[i];
        PointCloudT::Ptr ptr_cloud_out (new PointCloudT);
        reader.read(path_cloud, *ptr_cloud_out);
        pcl::transformPointCloud(*ptr_cloud_out, *ptr_cloud_out, poses_input_vec[i].matrix());

        icp.setInputSource(ptr_cloud_in);
        icp.setInputTarget(ptr_cloud_out);

        PointCloudT cloud_in_trans;
        // Eigen::Isometry3f init_guess = poses_input_vec[i].inverse() * poses_input_vec[i-1];
        Eigen::Isometry3f init_guess = poses_input_vec[i-1].inverse() * poses_input_vec[i];
        icp.align(cloud_in_trans, init_guess.matrix());

        if (icp.hasConverged()) 
            std::cout << "converged. The score is " << icp.getFitnessScore() << "\n";
        else 
            std::cout << "did not converge." << std::endl;

        Eigen::Matrix4f matrix_output = icp.getFinalTransformation();
        Eigen::Isometry3f pose_output;
        pose_output.matrix() = matrix_output;
        // poses_output_vec.push_back(pose_output * poses_output_vec.back());
        poses_output_vec.push_back(poses_output_vec.back() * pose_output);

        ptr_cloud_in = ptr_cloud_out;
    }

}

int main(int argc, char** argv) {
    std::string pc_folder {"/home/Autowise/data/autowise/lingang_map5/depth/depth_03/pcd_short/"};
    std::set<std::string> pc_list_set;
    if (!cch::loadPcdPath(pc_folder, pc_list_set)) {
        return 1;
    }
    std::vector<std::string> pc_vec (pc_list_set.begin(), pc_list_set.end());

    std::string path_to_input_pose {"/home/Autowise/data/autowise/lingang_map5/poses/gps/poses_cam.txt"};
    std::vector<Eigen::Isometry3f> poses_input_vec;
    std::vector<std::string> timestamps {};
    loadPoses(path_to_input_pose, poses_input_vec, timestamps);

    std::vector<Eigen::Isometry3f> poses_output_vec;
    // ndtOptimizePose(pc_vec, poses_input_vec, poses_output_vec);
    icpOptimizePose(pc_vec, poses_input_vec, poses_output_vec);

    std::string path_to_output_pose {"/home/Autowise/data/autowise/lingang_map5/poses/opt/poses_cam_icp_3.txt"};
    saveTransMatrix(path_to_output_pose, poses_output_vec, timestamps);

    return 0;
}