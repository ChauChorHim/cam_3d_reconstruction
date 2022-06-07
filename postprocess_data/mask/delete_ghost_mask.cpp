#include "files_handler.h"

#include <filesystem>
#include <iostream>
#include <set>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    if(argc != 3){
        std::cout << " Wrong number of arguments (!=2) " << std::endl;
    }

    std::string mask_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    if (false == cch::validateFolderDir(mask_folder_dir)) return 1;
    if (false == cch::validateFolderDir(output_folder_dir)) return 1;

    std::set<std::filesystem::path> mask_images_path;

    for (auto &path_to_file: std::filesystem::directory_iterator(mask_folder_dir))
        mask_images_path.insert(path_to_file.path());

    auto iter_mask_path = mask_images_path.begin();
    cv::Mat pre_mask = cv::imread(*iter_mask_path);

    iter_mask_path++;
    cv::Mat cur_mask = cv::imread(*iter_mask_path);

    iter_mask_path++;
    cv::Mat next_mask = cv::imread(*iter_mask_path);

    size_t index = 0;
    while (1) {
        index++;
        if (index % 1000 == 0) {
            std::cout << "Current index: " << index << " / " << mask_images_path.size() << "\n";
        }


        if (std::next(iter_mask_path) != mask_images_path.end())
            iter_mask_path++;
        else break;

        pre_mask = std::move(cur_mask);
        cur_mask = std::move(next_mask);
        next_mask = cv::imread(*iter_mask_path);
    }

    return 0;    
}