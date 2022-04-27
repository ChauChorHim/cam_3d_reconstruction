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

    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);


    cch::makeFilesList(input_folder_dir, output_folder_dir + "/image_files.txt");

    cch::splitFilesList(
        output_folder_dir + "/image_files.txt", 
        0.9, 
        output_folder_dir + "/train_files.txt", 
        output_folder_dir + "/val_files.txt");

    cch::shuffleList(output_folder_dir + "/train_files.txt");
    cch::shuffleList(output_folder_dir + "/val_files.txt");

    return 0;
}
