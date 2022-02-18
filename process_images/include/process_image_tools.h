#ifndef _UNDISTORT_H
#define _UNDISTORT_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <random>

class ImagesHandler {
    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        std::vector<std::string> images_path_buffer_;
    
    public:
        ImagesHandler();
        void readCameraParameters(const std::string &yml_filename);
        void loadImages(const std::string &input_folder_dir, const std::string &images_list);
        void undistortImages(const std::string &output_folder_dir);
        void cropImages(cv::Range &row_range, cv::Range &col_range, const std::string &output_folder_dir);
        void resizeImages(int height, int width, const std::string &output_folder_dir);

        static void makeImagesList(
            const std::string &path_to_images_folder, 
            const std::string &path_to_list="./images_list.txt", 
            bool re_order=false, 
            bool shuffle=false) {
                std::cout << "-> The path to images list is: " << path_to_list << std::endl;
                std::vector<std::string> images_list = {};
                int idx = 0;
                for (const auto& file : std::filesystem::directory_iterator(path_to_images_folder)) {
                    std::string image_list (file.path());
                    if (!re_order) { 
                        // Original file name
                        int loc_last_slash = image_list.find_last_of('/');
                        image_list = image_list.substr(loc_last_slash+1, image_list.size()-1);
                    } else {
                        // Modified file name 0.jpg, 1.jpg, 2.jgp, 3.jpg ...
                        int loc_last_dot = image_list.find_last_of('.');
                        std::stringstream ss;
                        ss << std::setw(6) << std::setfill('0') << idx++;
                        image_list = ss.str() + image_list.substr(loc_last_dot, image_list.size()-1);
                    }

                    images_list.push_back(image_list);
                }

                if(shuffle) {
                    auto rd = std::random_device {};
                    auto rng = std::default_random_engine {rd()};
                    std::shuffle(std::begin(images_list), std::end(images_list), rng);
                }

                std::ofstream list_file(path_to_list);
                std::ostream_iterator<std::string> list_iterator(list_file, "\n");
                std::copy(images_list.begin(), images_list.end(), list_iterator);
        }
};


#endif