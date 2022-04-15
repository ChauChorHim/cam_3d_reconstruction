#ifndef __IMAGES_HANDLER__
#define __IMAGES_HANDLER__

#include <vector>
#include <string>
#include <opencv2/core.hpp>

class ImagesHandler {
private:
    cv::Mat original_camera_matrix_;
    cv::Mat undistort_camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<std::string> images_path_buffer_;
    
public:
    ImagesHandler();
    void readCameraParameters(const std::string &yml_filename);
    void loadImages(const std::string &input_folder_dir, const std::string &images_list);
    void undistortImages(const std::string &output_folder_dir);
    void cropImages(const size_t row_1, const size_t row_2, const size_t col_1, const size_t col_2, const std::string &output_folder_dir);
    void resizeImages(const size_t height, const size_t width, const std::string &output_folder_dir);
    void undistortCropResizeImages(const std::string &output_folder_dir, const size_t num_images, const size_t row_1, const size_t row_2, const size_t col_1, const size_t col_2, const size_t height, const size_t width);
    void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow);

    void accessImagesBuffer(size_t index, std::string& path_to_image) const;
    size_t imagesNum() const;

private:
    
    void visualizeOpticalFlow(cv::Mat &angle, cv::Mat &magn_norm);
};

#endif