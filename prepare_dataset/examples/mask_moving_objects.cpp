/*

This file is for obtaining mask of moving objects given a rgb image and a instance segmentation mask

*/

#include "images_handler.h"
#include "opticalFlow_handler.h"
#include "npy.h"

template<class vecType>
void npy2vec(std::string data_fname, std::vector<vecType> &data_out) {
    std::vector<unsigned long> shape;
    bool is_fortran_order;

    shape.clear();
    data_out.clear();
    npy::LoadArrayFromNumpy(data_fname, shape, is_fortran_order, data_out);

    assert(false == is_fortran_order);
}

int main(int argc, char** argv) {
    if(argc != 4){
        std::cout << " Wrong number of arguments (!=3) " << std::endl;
    }

    std::string rgb_folder_dir(argv[1]);
    std::string mask_folder_dir(argv[2]);
    std::string output_folder_dir(argv[3]);

    if (false == cch::validateFolderDir(rgb_folder_dir)) return 1;
    if (false == cch::validateFolderDir(mask_folder_dir)) return 1;
    if (false == cch::validateFolderDir(output_folder_dir)) return 1;

    std::set<std::filesystem::path> rgb_images_path, mask_images_path;

    for (const auto &path_to_file: std::filesystem::directory_iterator(rgb_folder_dir))
        rgb_images_path.insert(path_to_file.path());

    for (const auto &path_to_file: std::filesystem::directory_iterator(mask_folder_dir))
        mask_images_path.insert(path_to_file.path());
    
    size_t index = 0;
    auto iter_rgb_path = rgb_images_path.begin();
    auto iter_mask_path = mask_images_path.begin();

    // assert(mask_images_path.end() - mask_images_path.begin() == rgb_images_path.end() - rgb_images_path.begin());

    bool is_initialized = false;
    cv::Mat pre_env_image;
    cv::Mat pre_non_env_image;
    while (iter_rgb_path != rgb_images_path.end()) {
        // Get current rgb, mask file path
        std::string cur_rgb_path = *iter_rgb_path;
        std::string cur_mask_path = *iter_mask_path;

        // Read current rgb image
        cv::Mat cur_rgb_image = cv::imread(cur_rgb_path);

        // Read current instance segmentation mask image
        std::vector<uchar> cur_mask_vec;
        npy2vec<uchar>(cur_mask_path, cur_mask_vec);
        cv::Mat1b cur_mask(cur_rgb_image.rows, cur_rgb_image.cols, cur_mask_vec.data());

        // Zeros mat used for masking the enviroment (environment id = 0)
        cv::Mat1b zeros = cv::Mat1b::zeros(cur_mask.rows, cur_mask.cols);

        // Get the current environment mask
        cv::Mat1b cur_env_mask = cv::Mat1b(cur_mask.rows, cur_mask.cols);
        cv::Mat1b cur_non_env_mask = cv::Mat1b(cur_mask.rows, cur_mask.cols);
        cv::compare(cur_mask, zeros, cur_env_mask, cv::CMP_EQ);
        cv::compare(cur_mask, zeros, cur_non_env_mask, cv::CMP_GT);
        
        cv::Mat cur_env_image;
        cv::Mat cur_non_env_image;
        cur_rgb_image.copyTo(cur_env_image, cur_env_mask);
        cur_rgb_image.copyTo(cur_non_env_image, cur_non_env_mask);
        // cv::imwrite("tmp/env_" + std::to_string(index) + ".jpg", cur_non_env_image);
        // cv::imwrite("tmp/rgb_" + std::to_string(index) + ".jpg", cur_rgb_image);

        // Initialize
        if (false == is_initialized) {
            is_initialized = true;
            pre_env_image = std::move(cur_env_image);
            pre_non_env_image = std::move(cur_non_env_image);

            continue;
        }


        // Compute the optical flow
        cv::Mat env_flow, non_env_flow;
        cch::getOpticalflow(pre_env_image, cur_env_image, env_flow);
        cch::getOpticalflow(pre_non_env_image, cur_non_env_image, non_env_flow);

        // Compute the optical flow magnitude
        double env_opticalFlow_magnitude = cch::computeFlowMagnitude(env_flow);
        double non_env_opticalFlow_magnitude = cch::computeFlowMagnitude(non_env_flow);
        std::cout << env_opticalFlow_magnitude << ": " << non_env_opticalFlow_magnitude << " = " << env_opticalFlow_magnitude / non_env_opticalFlow_magnitude << "\n";

        // update pre data
        pre_env_image = std::move(cur_env_image);
        pre_non_env_image = std::move(cur_non_env_image);

        // Move forward the iterator        
        iter_rgb_path = next(iter_rgb_path);
        iter_mask_path = next(iter_mask_path);
        index++;
    }

    return 0;
}