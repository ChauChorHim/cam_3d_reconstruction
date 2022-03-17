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

    files_handler.makeFilesList(input_folder_dir, output_folder_dir+"timestamps_files.txt", false, false, false, false);

    // filesHandler.makeFilesList(input_folder_dir, output_folder_dir+"depth_files.txt", false, false, true);

    // filesHandler.makeFilesList(input_folder_dir, output_folder_dir+"image_files.txt", false, true, false);
    // filesHandler.splitFilesList(output_folder_dir+"image_files.txt", 0.9, true, true);

    // filesHandler.makeFilesList(input_folder_dir, output_folder_dir+"/images_list.txt");
    // filesHandler.keepFirstNFiles(input_folder_dir, 300);
    // filesHandler.splitFilesList(output_folder_dir+"/images_list.txt", 0.9);
    
    return 0;
}
