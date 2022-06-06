#include "images_handler.h"
#include <iostream>

int main(int argc, char** argv){
    if(argc != 4){
        std::cout << " Wrong number of arguments (!=3) " << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string yml_filename(argv[1]);
    std::string input_folder_dir(argv[2]);
    std::string output_folder_dir(argv[3]);

    // Read the intrinsics
    cch::Camera camera_;
    camera_.load(yml_filename);

    cch::undistortImages(input_folder_dir, output_folder_dir, camera_);

    return 0;
}