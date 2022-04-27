/*

This file is for obtaining a list of removed static scenes

*/

#include "images_handler.h"
#include "files_handler.h"

#include <iostream>
#include <filesystem>
#include <iterator>
#include <fstream>

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " input_folder_dir" << " output_folder_dir" << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv){
    if(argc != 3){
        std::cout << " Wrong number of arguments (!=2) " << std::endl;
        printHelp(argv[0]);
    }

    cch::ImagesHandler imagesHandler = cch::ImagesHandler();
    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    std::string images_list {"./images_list_to_be_delete.txt"};
    cch::makeFilesList(input_folder_dir, images_list);

    imagesHandler.loadImages(input_folder_dir, images_list);

    std::string path_to_list = output_folder_dir + "/image_files.txt";
    if (std::filesystem::exists(path_to_list)) {
        std::remove(path_to_list.c_str());
    }
    std::vector<std::string> files_list = {};
    size_t i = 1;

    size_t images_num = imagesHandler.imagesNum();
    for (size_t index = 1; index < images_num; ++index) {
        std::string path_to_source_image, path_to_target_image;
        imagesHandler.accessImagesBuffer(index-1, path_to_source_image);
        imagesHandler.accessImagesBuffer(index, path_to_target_image);
        cv::Mat flow;
        imagesHandler.getOpticalflow(path_to_source_image, path_to_target_image, flow);

        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat magnitude, angle, magn_norm;
        cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        // angle *= ((1.f / 360.f) * (180.f / 255.f));

        double sum_of_magnitude = cv::sum(magnitude)[0];
        sum_of_magnitude /= (magnitude.size().width * magnitude.size().height);
        if (index % 1000 == 0)
            std::cout << "current index: " << index << ", opticalflow magnitude: " << sum_of_magnitude << "\n";

        if (sum_of_magnitude > 0.2) {
            // moving scene, save to a list
            auto loc_of_last_slash = path_to_target_image.find_last_of('/');
            std::string target_image_name = std::to_string(i++) + " ";
            target_image_name += path_to_target_image.substr(loc_of_last_slash + 1, path_to_target_image.size() - loc_of_last_slash);
            files_list.push_back(target_image_name);
        }
    }

    std::ofstream list_file(path_to_list);
    std::ostream_iterator<std::string> list_iterator(list_file, "\n");
    std::copy(files_list.begin(), files_list.end(), list_iterator);

    std::remove(images_list.c_str());

    return 0;
}