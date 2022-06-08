#include "files_handler.h"
#include "npy.h"

#include <vector>
#include <filesystem>
#include <set>
#include <string>

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

    std::string npy_mask_folder_dir(argv[1]);
    std::string jpeg_mask_folder_dir(argv[2]);

    if (false == cch::validateFolderDir(npy_mask_folder_dir)) return 1;
    if (false == cch::validateFolderDir(jpeg_mask_folder_dir)) return 1;

    std::set<std::filesystem::path> npy_mask_paths;

    for (const auto &path_to_file: std::filesystem::directory_iterator(npy_mask_folder_dir))
        npy_mask_paths.insert(path_to_file.path());

    
    auto iter_mask_path = npy_mask_paths.begin();

    std::string cur_mask_path = *iter_mask_path;
    std::vector<u_char> npy_mask_vec;
    auto npy_shape = npy2vec<u_char>(cur_mask_path, npy_mask_vec); // npy_shape [rows, cols]

    size_t index = 0;
    
    // Compute the threshold of small objects
    while (iter_mask_path != npy_mask_paths.end()) {
        index++;
        if (index % 1000 == 0)
            std::cout << "Current index: " << index << " / " << npy_mask_paths.size() << "\n";
        // Get current mask file path
        cur_mask_path = *iter_mask_path;

        // Read current instance segmentation mask image
        npy_mask_vec.clear();
        npy2vec<u_char>(cur_mask_path, npy_mask_vec);

        // Transfer the vector to a cv::Mat 
        cv::Mat1b mask_mat(npy_shape[0], npy_shape[1], npy_mask_vec.data());
        // cv::resize(mask_mat, mask_mat, cv::Size(704, 192), cv::INTER_NEAREST);
        std::string jpeg_mask_path = jpeg_mask_folder_dir + (*iter_mask_path).stem().c_str() + ".tiff";
        cv::imwrite(jpeg_mask_path, mask_mat);

        // Go ahead
        iter_mask_path = std::next(iter_mask_path);
    }

    return 0;
}