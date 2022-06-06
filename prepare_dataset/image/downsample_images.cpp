#include "images_handler.h"
#include <iostream>

int main(int argc, char** argv){
    if(argc != 5){
        std::cout << " Wrong number of arguments " << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);
    const size_t width(std::atoi(argv[3]));
    const size_t height(std::atoi(argv[4]));

    cch::downsampleImages(input_folder_dir, output_folder_dir, width, height);

    return 0;
}