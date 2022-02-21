#include "process_image_tools.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <opencv2/core/core.hpp>

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " camera.yml" << " input_folder_dir" << " images_list.xml " << " output_folder_dir" << std::endl;
    std::cout << " camera.yml = file with camera matrix and distortion coefficients. " << std::endl;
    std::cout << " images_paths.xml = file with path to images that should be undistorted." << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv){
    if(argc != 5){
        std::cout << " Wrong number of arguments (!=5) " << std::endl;
        printHelp(argv[0]);
    }

    ImagesHandler imagesHandler = ImagesHandler();
    const std::string yml_filename(argv[1]);
    std::string input_folder_dir(argv[2]);
    std::string images_list(argv[3]);
    std::string output_folder_dir(argv[4]);

    imagesHandler.readCameraParameters(yml_filename);
    imagesHandler.loadImages(input_folder_dir, images_list);
    imagesHandler.undistortImages(output_folder_dir);
    imagesHandler.loadImages(output_folder_dir, images_list);

    cv::Range cropped_height = cv::Range(0, 640);
    cv::Range cropped_width = cv::Range(0, 1280);
    imagesHandler.cropImages(cropped_height, cropped_width, output_folder_dir);
    int resized_height = 320;
    int resized_width = 640;
    imagesHandler.resizeImages(resized_height, resized_width, output_folder_dir);

    return 0;
}