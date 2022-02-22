#include "images_handler.h"
#include <iostream>
#include <string>
#include <cstdlib>

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

    imagesHandler.undistortCropResizeImages(output_folder_dir, 0, 640, 0, 1280, 320, 640);

    // imagesHandler.undistortImages(output_folder_dir);

    // imagesHandler.loadImages(output_folder_dir, images_list);
    // imagesHandler.cropImages(0, 640, 0, 1280, output_folder_dir);

    // imagesHandler.resizeImages(320, 640, output_folder_dir);

    return 0;
}