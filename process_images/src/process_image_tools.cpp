#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "process_image_tools.h"
#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>

// Read a YAML file with the camera parameters (cameraMatrix and distCoeffs)
void ImagesHandler::readCameraParameters(const std::string &yml_filename){
    std::cout << yml_filename << std::endl;
    cv::FileStorage fs(yml_filename, cv::FileStorage::READ);
    if( !fs.isOpened() ){
        std::cerr << " Fail to open " << yml_filename << std::endl;
        exit(EXIT_FAILURE);
    }

    // Get camera parameters
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_; 

    // Print out the camera parameters
    std::cout << "\n -- Camera parameters -- " << std::endl;
    std::cout << "\n CameraMatrix = " << std::endl << " " << camera_matrix_ << std::endl << std::endl;
    std::cout << " Distortion coefficients = " << std::endl << " " << dist_coeffs_ << std::endl << std::endl;

    fs.release();
}

// Load images path, images_list only has the name of images
void ImagesHandler::loadImages(const std::string &input_folder_dir, const std::string &images_list ){
    // Clear the buffer
    if (!images_path_buffer_.empty()) images_path_buffer_.clear();
    std::fstream fileHandler;
    fileHandler.open(images_list, std::ios::in);
    if (fileHandler.is_open()) {
        std::string curImageName; 
        while (getline(fileHandler, curImageName)) {
            images_path_buffer_.push_back(input_folder_dir + curImageName);
        }
        fileHandler.close();
    }
    std::cout << " Images number = " << images_path_buffer_.size() << std::endl;
}

// Undistort the images in the buffer
void ImagesHandler::undistortImages(const std::string &output_folder_dir){
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
        cv::undistort(inputImage, outputImage, camera_matrix_, dist_coeffs_);

        // Separate filename and path
        size_t found = imagePath->find_last_of("/");
        std::string path = imagePath->substr(0, found);
        std::string filename = imagePath->substr(found + 1);

        // Save undistorted image
        cv::imwrite(output_folder_dir + "/" + filename, outputImage);
    }
    std::cout << " Done !" << std::endl;
}

// Crop the images in the buffer
void ImagesHandler::cropImages(cv::Range &row_range, cv::Range &col_range, const std::string &output_folder_dir) {
    std::cout << " Cropping images... " << std::endl;

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
        std::string path = imagePath->substr(0, found);
        std::string filename = imagePath->substr(found + 1);

        // Save undistorted image
        cv::imwrite(output_folder_dir + "/" + filename, outputImage);

    }
    std::cout << " Done !" << std::endl;
}

void ImagesHandler::resizeImages(int height, int width, const std::string &output_folder_dir) {

    std::cout << " Resizing images... " << std::endl;

    for(auto imagePath = images_path_buffer_.begin(); imagePath != images_path_buffer_.end(); ++imagePath) {
        cv::Mat inputImage = cv::imread(*imagePath, cv::IMREAD_COLOR);
        if( !inputImage.data ){
            std::cout << " Could not open or find the image: " << *imagePath << std::endl;
            std::cout << " Verify if the input images path are absolute," << std::endl;
            std::cout << " or change the program directory." << std::endl;
            exit(EXIT_FAILURE);
        }
        cv::Mat outputImage;
        cv::resize(inputImage, outputImage, cv::Size(width, height), cv::INTER_LINEAR);

        // Separate filename and path
        size_t found = imagePath->find_last_of("/");
        std::string path = imagePath->substr(0, found);
        std::string filename = imagePath->substr(found + 1);

        // Save undistorted image
        cv::imwrite(output_folder_dir + "/" + filename, outputImage);

    }
    std::cout << " Done !" << std::endl;
}

ImagesHandler::ImagesHandler() {

}
