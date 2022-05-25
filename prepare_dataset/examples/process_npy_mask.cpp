#include "npy.h"
#include "files_handler.h"

#include <filesystem>
#include <cassert>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

template<class vecType>
std::vector<unsigned long> npy2vec(std::string data_fname, std::vector<vecType> &data_out) {
    std::vector<unsigned long> shape;
    bool is_fortran_order;

    shape.clear();
    data_out.clear();
    npy::LoadArrayFromNumpy(data_fname, shape, is_fortran_order, data_out);

    assert(false == is_fortran_order);

    return shape;
}

int main(int argc, char** argv) {
    if(argc != 3){
        std::cout << " Wrong number of arguments (!=2) " << std::endl;
    }

    std::string mask_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    if (false == cch::validateFolderDir(mask_folder_dir)) return 1;
    if (false == cch::validateFolderDir(output_folder_dir)) return 1;

    std::set<std::filesystem::path> mask_images_path;

    for (const auto &path_to_file: std::filesystem::directory_iterator(mask_folder_dir))
        mask_images_path.insert(path_to_file.path());

    /* Compute the threshold size of objects */

    auto iter_mask_path = mask_images_path.begin();

    // There are at most 25 objects in a mask, the i-th element of the vector is the pixel number of the i-th object
    std::vector<int> objects_pixel_num(25, 0);

    std::string cur_mask_path = *iter_mask_path;
    std::vector<u_char> cur_mask_vec;
    auto npy_shape = npy2vec<u_char>(cur_mask_path, cur_mask_vec); // npy_shape [rows, cols]

    std::unordered_map<int, int> map_area_num;

    size_t index = 0;
    
    // Compute the threshold of small objects
    // while (iter_mask_path != mask_images_path.end()) {
    //     index++;
    //     if (index % 1000 == 0)
    //         std::cout << "Current index: " << index << " / " << mask_images_path.size() << "\n";
    //     // Get current mask file path
    //     cur_mask_path = *iter_mask_path;

    //     // Read current instance segmentation mask image
    //     cur_mask_vec.clear();
    //     npy2vec<u_char>(cur_mask_path, cur_mask_vec);

    //     // Count the number of pixels for each object 
    //     std::fill(objects_pixel_num.begin(), objects_pixel_num.end(), 0);
    //     for (u_char pixel: cur_mask_vec) {
    //         if (pixel == 0) continue; // skip the environment (non-object)
    //         objects_pixel_num[pixel]++;
    //     }

    //     for (int ele : objects_pixel_num) {
    //         if (ele != 0)
    //             map_area_num[ele]++;
    //     } 

    //     // Go ahead
    //     iter_mask_path = std::next(iter_mask_path);
    // }

    // // Count the number of detected objects in all masks
    // int objects_number = 0;
    // for (auto &ele: map_area_num) {
    //     objects_number += ele.second;
    // }
    // std::cout << "The number of all detected objects in all masks: " << objects_number << " (pixels)\n";

    // // Compute the threshold for filtering small objects
    // int cur_objects_number = 0;
    // int thres_object_area;
    // for (auto &ele: map_area_num) {
    //     cur_objects_number += ele.second;
    //     if (cur_objects_number > 0.9 * objects_number) {
    //         thres_object_area = ele.first;
    //         break;
    //     }
    // }
    // std::cout << "Threshold of object area (above 90 percentage objects): " << thres_object_area << "\n";

    int thres_object_area = 4216;

    // Filter out object area small than thres_object_area

    std::cout << "\nStarting remove the objects areas smaller than: " << thres_object_area << "\n";
    cur_mask_path = *iter_mask_path;
    index = 0;
    
    while (iter_mask_path != mask_images_path.end()) {
        index++;
        if (index % 1000 == 0)
            std::cout << "Current index: " << index << " / " << mask_images_path.size() << "\n";
        // Get current mask file path
        cur_mask_path = *iter_mask_path;

        // Read current instance segmentation mask image
        cur_mask_vec.clear();
        npy2vec<u_char>(cur_mask_path, cur_mask_vec);

        // Count the number of pixels for each object 
        std::fill(objects_pixel_num.begin(), objects_pixel_num.end(), 0);
        for (u_char pixel: cur_mask_vec) {
            if (pixel == 0) continue; // skip the environment (non-object)
            objects_pixel_num[pixel]++;
        }

        // Loop every object and tag the mask value of small object
        std::unordered_set<int> small_object_tag;
        for (size_t i = 1; i < objects_pixel_num.size(); ++i) { // exclude the env
            if (objects_pixel_num[i] < thres_object_area) {
                small_object_tag.insert(i);
            }
        }

        for (size_t i = 0; i < cur_mask_vec.size(); ++i) {
            u_char &pixel = cur_mask_vec[i];
            if (pixel == 0) continue;
            if (small_object_tag.find(pixel) != small_object_tag.end()) {
                pixel = 0;
            } else {
                pixel = 255;
            }
        }

        // Transfer the vector to a cv::Mat 
        cv::Mat1b cur_mask(npy_shape[0], npy_shape[1], cur_mask_vec.data());
        cv::resize(cur_mask, cur_mask, cv::Size(704, 192), cv::INTER_LINEAR);
        std::string path_new_mask = output_folder_dir + (*iter_mask_path).stem().c_str() + ".jpeg";
        cv::imwrite(path_new_mask, cur_mask);
        
        // Go ahead
        iter_mask_path = std::next(iter_mask_path);
    }

    return 0;
}