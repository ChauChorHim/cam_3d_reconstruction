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

    FilesHandler filesHandler = FilesHandler();
    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    filesHandler.makeFilesList(input_folder_dir, output_folder_dir+"images_list.txt", false);
    filesHandler.splitFilesList(output_folder_dir+"images_list.txt", 0.9);

    // filesHandler.makeFilesList(input_folder_dir, output_folder_dir+"/images_list.txt");
    // filesHandler.keepFirstNFiles(input_folder_dir, 50000);
    // filesHandler.splitFilesList(output_folder_dir+"/images_list.txt", 0.9);
    
    return 0;
}