#include "files_handler.h"

#include <filesystem>
#include <cassert>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>
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

    /* Compute the threshold size of objects */

    auto iter_mask_path = mask_images_path.begin();
    cv::Mat mask_dummy = cv::imread(*iter_mask_path, cv::IMREAD_GRAYSCALE);
    auto mask_shape = mask_dummy.size();
    size_t width = mask_dummy.cols;
    size_t height = mask_dummy.rows;

    // There are at most 25 objects in a mask, the i-th element of the vector is the pixel number of the i-th object
    std::vector<int> objects_pixel_num(25, 0);

    // Divide potential area into serveral bins
    std::map<int, int> area_num;
    int total_area = width * height;
    int num_bins = 100;
    int max_object_area = total_area / 10;
    for (int i = 0; i <= num_bins; ++i) {
        int area = i * max_object_area / num_bins;
        area_num[area] = 0;
    }

    size_t index = 0;
    // Compute the threshold of small objects
    while (1) {
        index++;
        if (index % 1000 == 0) {
            std::cout << "Current index: " << index << " / " << mask_images_path.size() << "\n";
        }

        cv::Mat cur_mask = cv::imread(*iter_mask_path, cv::IMREAD_GRAYSCALE);

        // Count the number of pixels for each object 
        std::fill(objects_pixel_num.begin(), objects_pixel_num.end(), 0);
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                int pixel = cur_mask.at<uchar>(i, j);
                if (pixel == 0) continue; // skip the environment (non-object)
                objects_pixel_num[pixel]++;
            }
        }

        // insert object area into the map
        for (int ele : objects_pixel_num) {
            if (ele == 0) continue;
            auto lower_bound = area_num.lower_bound(ele);
            if (lower_bound == area_num.end()) {
                prev(lower_bound)->second++;
                continue;
            }
            lower_bound->second++;
        } 
        if (std::next(iter_mask_path) != mask_images_path.end())
            iter_mask_path++;
        else break;
    }

    int num_objects = 0;
    for (auto &ele: area_num) {
        num_objects += ele.second;
    }
    float thres = 0.9;
    int thres_objects_num = thres * num_objects;
    int thres_area = 0;

    for (auto &ele: area_num) {
        if (thres_objects_num - ele.second > 0) {
            thres_objects_num -= ele.second;
        } else {
            thres_area = ele.first;
            break;
        }
    }

    std::cout << "Threshold area: " << thres_area << std::endl;


    // Filter out object area small than thres_object_area

    std::cout << "\nStarting remove the objects areas smaller than: " << thres_area << "\n";

    index = 0;
    iter_mask_path = mask_images_path.begin();    
    while (1) {
        index++;
        if (index % 1000 == 0)
            std::cout << "Current index: " << index << " / " << mask_images_path.size() << "\n";
        
        cv::Mat cur_mask = cv::imread(*iter_mask_path, cv::IMREAD_GRAYSCALE);

        // Count the number of pixels for each object 
        std::fill(objects_pixel_num.begin(), objects_pixel_num.end(), 0);
        for (size_t i = 0; i < height; ++i) {
            for (size_t j = 0; j < width; ++j) {
                int pixel = cur_mask.at<uchar>(i, j);
                if (pixel == 0) continue; // skip the environment (non-object)
                objects_pixel_num[pixel]++;
            }
        }

        // Loop every object and tag the mask value of big object
        std::unordered_set<int> big_object_tag;
        for (size_t i = 1; i < objects_pixel_num.size(); ++i) { // exclude the env
            if (objects_pixel_num[i] >= thres_area) {
                big_object_tag.insert(i);
            }
        }

        cv::Mat mask_to_save = cur_mask.clone();
        // Loop every pixel and assign 0 for small object pixel
        for (size_t i = 0; i < height; ++i) {
            for (size_t j = 0; j < width; ++j) {
                int pixel = mask_to_save.at<uchar>(i, j);
                if (pixel == 0) continue; // skip the environment
                if (big_object_tag.find(pixel) != big_object_tag.end()) {
                    // if it is a big object, assign what value?
                    mask_to_save.at<uchar>(i, j) = 255; 
                } else {
                    mask_to_save.at<uchar>(i, j) = 0;
                }
            }
        }

        std::string path_new_mask = output_folder_dir + (*iter_mask_path).stem().c_str() + ".jpeg";
        cv::imwrite(path_new_mask, mask_to_save);
        
        if (std::next(iter_mask_path) != mask_images_path.end())
            iter_mask_path = std::next(iter_mask_path);
        else break;
    }

    return 0;
}