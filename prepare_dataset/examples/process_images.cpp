#include "images_handler.h"
#include "files_handler.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <cstdio>

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " camera.yml" << " input_folder_dir" << " output_folder_dir" << std::endl;
    std::cout << " camera.yml = file with camera matrix and distortion coefficients. " << std::endl;
    std::cout << " images_paths.xml = file with path to images that should be undistorted." << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv){
    if(argc != 4){
        std::cout << " Wrong number of arguments (!=5) " << std::endl;
        printHelp(argv[0]);
    }

    cch::ImagesHandler imagesHandler = cch::ImagesHandler();
    std::string yml_filename(argv[1]);
    std::string input_folder_dir(argv[2]);
    std::string output_folder_dir(argv[3]);

    // Make a temporary file list (to be delted at the end)
    std::string images_list {"./images_list_to_be_delete.txt"};
    cch::makeFilesList(input_folder_dir, images_list);

    // Read the intrinsics
    imagesHandler.readCameraParameters(yml_filename);

    imagesHandler.loadImages(input_folder_dir, images_list);
    // imagesHandler.undistortImages(output_folder_dir);
    imagesHandler.cropImages(576, 1536, 600, 960, output_folder_dir); // manydepth cityscape size

    // Delete the file list
    std::remove(images_list.c_str());

    return 0;
}