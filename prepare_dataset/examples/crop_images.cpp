#include "images_handler.h"

int main(int argc, char** argv){
    if(argc != 8){
        std::cout << " Wrong number of arguments (!=7) " << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string yml_filename(argv[1]);
    std::string input_folder_dir(argv[2]);
    std::string output_folder_dir(argv[3]);
    const size_t height(std::atoi(argv[4]));
    const size_t width(std::atoi(argv[5]));
    const size_t row_crop_center(std::atoi(argv[6]));
    const size_t col_crop_center(std::atoi(argv[7]));

    // Read the intrinsics
    cch::Camera camera_;
    camera_.load(yml_filename);

    cch::cropImages(input_folder_dir, output_folder_dir, camera_, height, width, row_crop_center, col_crop_center);

    return 0;
}