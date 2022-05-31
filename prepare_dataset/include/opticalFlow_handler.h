#ifndef __OPTICALFOLW_HANDLER_
#define __OPTICALFOLW_HANDLER_

#include <opencv2/core.hpp>
#include <string>

namespace cch
{
void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow, float resize_factor = 0.25, bool show_opticalflow = false);
void getOpticalflow(const cv::Mat &source_image, const cv::Mat &target_image, cv::Mat &flow, float resize_factor = 0.25, bool show_opticalflow = false);
void visualizeOpticalFlow(cv::Mat &flow);
double computeFlowMagnitude(cv::Mat &flow, bool normalize_full = true);
   
} // namespace cch


#endif