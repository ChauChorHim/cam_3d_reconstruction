#include "files_handler.h"
#include <iostream>
#include <string>
#include <cstdlib>

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " input_folder_dir" << " output_folder_dir" << std::endl;
    std::cout << " camera.yml = file with camera matrix and distortion coefficients. " << std::endl;
    std::cout << " images_paths.xml = file with path to images that should be undistorted." << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv){
    if(argc != 3){
        std::cout << " Wrong number of arguments" << std::endl;
        printHelp(argv[0]);
    }

    FilesHandler files_handler = FilesHandler();
    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    /* make train_files.txt, val_files.txt; Carefule: don't shuffle when making the image_files.txt */
    // files_handler.makeFilesList(input_folder_dir, output_folder_dir+"image_files.txt", false);
    // files_handler.splitFilesList(output_folder_dir+"image_files.txt", 0.9, true, true);

    /* delete files */
    files_handler.keepFirstNFiles(input_folder_dir, 300);
    
    return 0;
}
