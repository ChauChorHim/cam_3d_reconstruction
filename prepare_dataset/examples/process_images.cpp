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


    /* full size lingang_map5/cam00 setting */
    // imagesHandler.loadImages(input_folder_dir, images_list);
    // imagesHandler.undistortCropResizeImages(output_folder_dir, 24833, 1080/2, 1080/2+32*10, 0+32*4, 1920-32*4, 320, 1920-32*8);

    /* full size distorted lingang_map5/cam00 setting */
    // imagesHandler.loadImages(input_folder_dir, images_list);
    // imagesHandler.cropImages(120, 1080, 0, 1920, output_folder_dir);

    /* full size undistorted lingang_map5/cam00 setting */
    imagesHandler.loadImages(input_folder_dir, images_list);
    imagesHandler.undistortCropResizeImages(output_folder_dir, 24828, 120, 952, 0 + 32*4, 1920-32*4, 832, 1664);

    // Delete the file list
    std::remove(images_list.c_str());

    return 0;
}