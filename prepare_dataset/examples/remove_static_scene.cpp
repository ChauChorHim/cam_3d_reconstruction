/*

This file is for obtaining a list of removed static scenes using optical flow

*/

#include "images_handler.h"
#include "opticalFlow_handler.h"

int main(int argc, char** argv){
    if(argc != 3){
        std::cout << " Wrong number of arguments (!=2) " << std::endl;
    }

    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    if (false == cch::validateFolderDir(input_folder_dir)) return 1;
    if (false == cch::validateFolderDir(output_folder_dir)) return 1;

    std::string path_to_list = output_folder_dir + "image_files_nonstatic.txt";
    if (std::filesystem::exists(path_to_list)) {
        std::remove(path_to_list.c_str());
    }

    std::set<std::filesystem::path> sorted_by_name;

    for (const auto &path_to_file: std::filesystem::directory_iterator(input_folder_dir))
        sorted_by_name.insert(path_to_file.path());

    std::vector<std::string> files_list = {};
    std::string file_list {};

    size_t index = 0;
    auto iter_to_path = sorted_by_name.begin();
    while (next(iter_to_path) != sorted_by_name.end()) {
        std::string path_to_source_image = *iter_to_path;
        std::string path_to_target_image = *(next(iter_to_path));

        cv::Mat flow;
        cch::getOpticalflow(path_to_source_image, path_to_target_image, flow);
        double sum_of_magnitude = cch::computeFlowMagnitude(flow);
        
        if (index % 100 == 0)
            std::cout << "current index: " << index << ", opticalflow magnitude: " << sum_of_magnitude << "\n";

        if (sum_of_magnitude > 0.2) {
            // moving scene, save to a list
            std::string filename = (*iter_to_path).filename();
            std::string valid_file_line = std::to_string(index++) + " " + filename;
            files_list.push_back(valid_file_line);
        }

        iter_to_path++;
    }

    std::ofstream list_file(path_to_list);
    std::ostream_iterator<std::string> list_iterator(list_file, "\n");
    std::copy(files_list.begin()+2, files_list.end()-2, list_iterator);

    return 0;
}