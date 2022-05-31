#ifndef __IMAGES_HANDLER__
#define __IMAGES_HANDLER__

#include <opencv2/core.hpp>

namespace cch {

struct Camera {
public:
    cv::Mat original_camera_matrix_;
    cv::Mat undistort_camera_matrix_;
    cv::Mat dist_coeffs_;
    void load(const std::string &file);
};

void undistortImages(std::string &input_folder_dir, std::string &output_folder_dir, Camera &camera_);
Camera cropImages(std::string &input_folder_dir, std::string &output_folder_dir, Camera& orignal_camera, const size_t height, const size_t width, const size_t row_crop_center, const size_t col_crop_center);


};

#endif