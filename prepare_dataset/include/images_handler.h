#ifndef __IMAGES_HANDLER__
#define __IMAGES_HANDLER__

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video.hpp>

#include "files_handler.h"

namespace cch {

struct Camera {
public:
    cv::Mat original_camera_matrix_;
    cv::Mat undistort_camera_matrix_;
    cv::Mat dist_coeffs_;
    void load(const std::string &file);
};

void undistortImages(std::string &input_folder_dir, std::string &output_folder_dir, Camera &camera_);
Camera cropImages(std::string &input_folder_dir, std::string &output_folder_dir, Camera& orignal_camera, const size_t height, const size_t width, const size_t row_crop_center, const size_t col_crop_center);

void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow);
void visualizeOpticalFlow(cv::Mat &angle, cv::Mat &magn_norm);

/* --------------------------------------------------------------------------------- */

// Read a YAML file with the camera parameters (cameraMatrix and distCoeffs)
void Camera::load(const std::string &file){
    std::cout << "Loading the camera parameters from: " << file << std::endl;
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if( !fs.isOpened() ){
        std::cerr << " Fail to open " << file << std::endl;
        exit(EXIT_FAILURE);
    }

    // Get camera parameters
    fs["camera_matrix"] >> original_camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_; 

    undistort_camera_matrix_ = cv::getOptimalNewCameraMatrix(
        original_camera_matrix_, 
        dist_coeffs_, 
        cv::Size(fs["image_width"], fs["image_height"]), 
        1, 
        cv::Size(fs["image_width"], fs["image_height"])
    );

    // Print out the camera parameters
    std::cout << "\n -- Camera parameters -- " << std::endl;
    std::cout << "\n Original CameraMatrix = " << std::endl << " " << original_camera_matrix_ << std::endl << std::endl;
    std::cout << "\n Distortion coefficients = " << std::endl << " " << dist_coeffs_ << std::endl << std::endl;

    fs.release();
}

void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow) {
    cv::Mat source_image = cv::imread(path_to_source_image);
    cv::Mat target_image = cv::imread(path_to_target_image);
    assert(!source_image.empty() && !target_image.empty());

    cv::cvtColor(source_image, source_image, cv::COLOR_BGR2GRAY);
    cv::cvtColor(target_image, target_image, cv::COLOR_BGR2GRAY);

    cv::resize(source_image, source_image, cv::Size(), 0.25, 0.25, cv::INTER_CUBIC);
    cv::resize(target_image, target_image, cv::Size(), 0.25, 0.25, cv::INTER_CUBIC);

    source_image = source_image(cv::Range(0, source_image.size().height/2), cv::Range(0, source_image.size().width));
    target_image = target_image(cv::Range(0, target_image.size().height/2), cv::Range(0, target_image.size().width));

    cv::Mat flow_(source_image.size(), CV_32FC2);

    cv::calcOpticalFlowFarneback(source_image, target_image, flow_, 0.5, 3, 15, 3, 5, 1.2, 0);

    flow = std::move(flow_);

    bool show_opticalflow = false;
    if (show_opticalflow) {
        // visualization
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat magnitude, angle, magn_norm;
        cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        // angle *= ((1.f / 360.f) * (180.f / 255.f));

        double sum_of_magnitude = cv::sum(magnitude)[0];
        sum_of_magnitude /= (magnitude.size().width * magnitude.size().height);
        std::cout << sum_of_magnitude << "\n";
        visualizeOpticalFlow(angle, magn_norm);
    }
}

void undistortImages(std::string &input_folder_dir, std::string &output_folder_dir, Camera &camera_) {
    // Validate the directory of input folder and output folder
    if (false == cch::validateFolderDir(input_folder_dir)) return;
    if (false == cch::validateFolderDir(output_folder_dir)) return;

    std::filesystem::directory_iterator list(input_folder_dir);

    std::cout << "\n Undistort CameraMatrix = " << std::endl << " " << camera_.undistort_camera_matrix_ << std::endl << std::endl;
    std::cout << " Undistorting images... " << std::endl;

    for (auto& it: list) {
        std::string filename = it.path().filename();
        cv::Mat inputImage = cv::imread(it.path(), cv::IMREAD_COLOR);
        cv::Mat outputImage;
        cv::undistort(inputImage, outputImage, camera_.original_camera_matrix_, camera_.dist_coeffs_, camera_.undistort_camera_matrix_);

        cv::imwrite(output_folder_dir + filename, outputImage);
    }

    std::cout << " Done !" << std::endl;
}

Camera cropImages(std::string &input_folder_dir, std::string &output_folder_dir, Camera& orignal_camera, const size_t height, const size_t width, const size_t row_crop_center, const size_t col_crop_center) {
    // Validate the directory of input folder and output folder
    if (false == cch::validateFolderDir(input_folder_dir)) return orignal_camera;
    if (false == cch::validateFolderDir(output_folder_dir)) return orignal_camera;

    std::filesystem::directory_iterator list(input_folder_dir);
    
    Camera cropped_camera = orignal_camera;

    cropped_camera.undistort_camera_matrix_.at<double>(0, 2) += (double(width) / 2 - col_crop_center);
    cropped_camera.undistort_camera_matrix_.at<double>(1, 2) += (double(height) / 2 - row_crop_center);

    cv::Range row_range = cv::Range(row_crop_center - height / 2, row_crop_center + height / 2);
    cv::Range col_range = cv::Range(col_crop_center - width / 2, col_crop_center + width / 2);

    std::cout << "\n Cropped CameraMatrix = \n " << cropped_camera.undistort_camera_matrix_ << "\n\n";
    std::cout << " Cropping images... \n";

    for (auto& it: list) {
        std::string filename = it.path().filename();
        cv::Mat inputImage = cv::imread(it.path(), cv::IMREAD_COLOR);
        cv::Mat outputImage;
        outputImage = inputImage(row_range, col_range);

        cv::imwrite(output_folder_dir + filename, outputImage);
    }

    std::cout << " Done !" << std::endl;

    return cropped_camera;
}

void visualizeOpticalFlow(cv::Mat &angle, cv::Mat &magn_norm) {
    // build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
    cv::imshow("frame2", bgr);
    cv::waitKey(0);
}

};

#endif