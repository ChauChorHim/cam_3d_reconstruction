#ifndef __IMAGES_HANDLER__
#define __IMAGES_HANDLER__

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video.hpp>

namespace cch {

class ImagesHandler {
private:
    cv::Mat original_camera_matrix_;
    cv::Mat undistort_camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<std::string> images_path_buffer_;
    
public:
    ImagesHandler();
    void readCameraParameters(const std::string &yml_filename);
    void loadImages(std::string &input_folder_dir, const std::string &images_list);
    void undistortImages(std::string &output_folder_dir);
    void cropImages(const size_t height, const size_t width, const size_t row_crop_center, const size_t col_crop_center, std::string &output_folder_dir);

    void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow);
    void accessImagesBuffer(size_t index, std::string& path_to_image) const;
    size_t imagesNum() const;

private:
    void visualizeOpticalFlow(cv::Mat &angle, cv::Mat &magn_norm);
};

/* --------------------------------------------------------------------------------- */


ImagesHandler::ImagesHandler() {

}

// Read a YAML file with the camera parameters (cameraMatrix and distCoeffs)
void ImagesHandler::readCameraParameters(const std::string &yml_filename){
    std::cout << yml_filename << std::endl;
    cv::FileStorage fs(yml_filename, cv::FileStorage::READ);
    if( !fs.isOpened() ){
        std::cerr << " Fail to open " << yml_filename << std::endl;
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

// Load images path
void ImagesHandler::loadImages(std::string &input_folder_dir, const std::string &images_list){
    if (input_folder_dir.back() != '/') {
        input_folder_dir.push_back('/');
    }
    // Clear the buffer
    if (!images_path_buffer_.empty()) images_path_buffer_.clear();
    std::fstream fileHandler;
    fileHandler.open(images_list, std::ios::in);
    if (fileHandler.is_open()) {
        std::string file_line; 
        while (getline(fileHandler, file_line)) {
            std::string cur_image_name (std::find(file_line.begin(), file_line.end(), ' ') + 1, file_line.end());
            images_path_buffer_.push_back(input_folder_dir + cur_image_name);
        }
        fileHandler.close();
    }
    std::cout << " Images number = " << images_path_buffer_.size() << std::endl;
}

void ImagesHandler::getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow) {
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

// Undistort the images in the buffer
void ImagesHandler::undistortImages(std::string &output_folder_dir){
    if (output_folder_dir.back() != '/') {
        output_folder_dir.push_back('/');
    }

    std::cout << "\n Undistort CameraMatrix = " << std::endl << " " << undistort_camera_matrix_ << std::endl << std::endl;
    std::cout << " Undistorting images... " << std::endl;

    for(auto imagePath = images_path_buffer_.begin(); imagePath != images_path_buffer_.end(); ++imagePath) {
        cv::Mat inputImage = cv::imread(*imagePath, cv::IMREAD_COLOR);
        if( !inputImage.data ){
            std::cout << " Could not open or find the image: " << *imagePath << std::endl;
            std::cout << " Verify if the input images path are absolute," << std::endl;
            std::cout << " or change the program directory." << std::endl;
            exit(EXIT_FAILURE);
        }
        cv::Mat outputImage;
        cv::undistort(inputImage, outputImage, original_camera_matrix_, dist_coeffs_, undistort_camera_matrix_);

        // Separate filename and path
        size_t found = imagePath->find_last_of("/");
        std::string filename = imagePath->substr(found + 1);

        // Save undistorted image
        cv::imwrite(output_folder_dir + filename, outputImage);
    }
    std::cout << " Done !" << std::endl;
}

// Crop the images in the buffer
void ImagesHandler::cropImages(const size_t height, const size_t width, const size_t row_crop_center, const size_t col_crop_center, std::string &output_folder_dir) {
    if (output_folder_dir.back() != '/') {
        output_folder_dir.push_back('/');
    }

    undistort_camera_matrix_.at<double>(0, 2) += (double(width) / 2 - col_crop_center);
    undistort_camera_matrix_.at<double>(1, 2) += (double(height) / 2 - row_crop_center);
    std::cout << "\n Cropped CameraMatrix = " << std::endl << " " << undistort_camera_matrix_ << std::endl << std::endl;
    std::cout << " Cropping images... " << std::endl;

    cv::Range row_range = cv::Range(row_crop_center - height / 2, row_crop_center + height / 2);
    cv::Range col_range = cv::Range(col_crop_center - width / 2, col_crop_center + width / 2);

    for(auto imagePath = images_path_buffer_.begin(); imagePath != images_path_buffer_.end(); ++imagePath) {
        cv::Mat inputImage = cv::imread(*imagePath, cv::IMREAD_COLOR);
        if( !inputImage.data ){
            std::cout << " Could not open or find the image: " << *imagePath << std::endl;
            std::cout << " Verify if the input images path are absolute," << std::endl;
            std::cout << " or change the program directory." << std::endl;
            exit(EXIT_FAILURE);
        }
        cv::Mat outputImage;
        outputImage = inputImage(row_range, col_range);

        // Separate filename and path
        size_t found = imagePath->find_last_of("/");
        std::string filename = imagePath->substr(found + 1);

        // Save cropped image
        cv::imwrite(output_folder_dir + filename, outputImage);
    }

    std::cout << " Done !" << std::endl;
}

void ImagesHandler::accessImagesBuffer(size_t index, std::string& path_to_image) const {
    path_to_image = images_path_buffer_[index];
}

size_t ImagesHandler::imagesNum() const {
    return images_path_buffer_.size();
}

void ImagesHandler::visualizeOpticalFlow(cv::Mat &angle, cv::Mat &magn_norm) {
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