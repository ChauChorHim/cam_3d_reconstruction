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

    ImagesHandler imagesHandler = ImagesHandler();
    std::string yml_filename(argv[1]);
    std::string input_folder_dir(argv[2]);
    std::string output_folder_dir(argv[3]);

    // Make a temporary file list (to be delted at the end)
    std::string images_list {"./images_list_to_be_delete.txt"};
    FilesHandler filesHandler = FilesHandler();
    filesHandler.makeFilesList(input_folder_dir, images_list, false);

    // Read the intrinsics
    imagesHandler.readCameraParameters(yml_filename);


    /* lingang_map4/cam00 setting */
    // imagesHandler.loadImages(input_folder_dir, images_list);
    // imagesHandler.undistortCropResizeImages(output_folder_dir, 0, 640, 0, 1280, 320, 640);

    /* lingang_map5/cam00 setting   lingang_map5/cam04 setting */
    imagesHandler.loadImages(input_folder_dir, images_list);
    imagesHandler.undistortCropResizeImages(output_folder_dir, 60, 1020, 0, 1920, 320, 640);

    // Delete the file list
    std::remove(images_list.c_str());

    return 0;
}