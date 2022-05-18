#ifndef __OPTICALFOLW_HANDLER_
#define __OPTICALFOLW_HANDLER_

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video.hpp>

#include <string>

namespace cch
{
void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow, float resize_factor = 0.25, bool show_opticalflow = false);
void getOpticalflow(const cv::Mat &source_image, const cv::Mat &target_image, cv::Mat &flow, float resize_factor = 0.25, bool show_opticalflow = false);
void visualizeOpticalFlow(cv::Mat &flow);
double computeFlowMagnitude(cv::Mat &flow, bool normalize_full = true);

/* --------------------------------------------------------------------------------- */

void getOpticalflow(const std::string &path_to_source_image, const std::string &path_to_target_image, cv::Mat &flow,
                    float resize_factor, bool show_opticalflow) {
    cv::Mat source_image = cv::imread(path_to_source_image);
    cv::Mat target_image = cv::imread(path_to_target_image);
    assert(!source_image.empty() && !target_image.empty());

    cv::cvtColor(source_image, source_image, cv::COLOR_BGR2GRAY);
    cv::cvtColor(target_image, target_image, cv::COLOR_BGR2GRAY);

    cv::resize(source_image, source_image, cv::Size(), resize_factor, resize_factor, cv::INTER_CUBIC);
    cv::resize(target_image, target_image, cv::Size(), resize_factor, resize_factor, cv::INTER_CUBIC);

    source_image = source_image(cv::Range(0, source_image.size().height/2), cv::Range(0, source_image.size().width));
    target_image = target_image(cv::Range(0, target_image.size().height/2), cv::Range(0, target_image.size().width));

    cv::Mat flow_(source_image.size(), CV_32FC2);

    cv::calcOpticalFlowFarneback(source_image, target_image, flow_, 0.5, 3, 15, 3, 5, 1.2, 0);

    flow = std::move(flow_);

    if (show_opticalflow) {
        visualizeOpticalFlow(flow);
    }
}

void getOpticalflow(const cv::Mat &source_image, const cv::Mat &target_image, cv::Mat &flow, 
                    float resize_factor, bool show_opticalflow) {
    cv::Mat resize_source_image, resize_target_image;
    cv::resize(source_image, resize_source_image, cv::Size(), resize_factor, resize_factor, cv::INTER_CUBIC);
    cv::resize(target_image, resize_target_image, cv::Size(), resize_factor, resize_factor, cv::INTER_CUBIC);

    resize_source_image = resize_source_image(cv::Range(0, resize_source_image.rows/2), cv::Range(0, resize_source_image.cols));
    resize_target_image = resize_target_image(cv::Range(0, resize_target_image.rows/2), cv::Range(0, resize_target_image.cols));

    cv::cvtColor(resize_source_image, resize_source_image, cv::COLOR_BGR2GRAY);
    cv::cvtColor(resize_target_image, resize_target_image, cv::COLOR_BGR2GRAY);

    cv::Mat flow_(cv::Size(resize_source_image.cols, resize_source_image.rows), CV_32FC2);

    cv::calcOpticalFlowFarneback(resize_source_image, resize_target_image, flow_, 0.5, 3, 15, 3, 5, 1.2, 0);

    flow = std::move(flow_);

    if (show_opticalflow) {
        visualizeOpticalFlow(flow);
    }
}

void visualizeOpticalFlow(cv::Mat &flow) {
    cv::Mat flow_parts[2];
    cv::split(flow, flow_parts);
    cv::Mat magnitude, angle, magn_norm;
    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    // build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
    cv::imshow("frame2", bgr);
    cv::waitKey(0);
    // cv::imwrite("tmp/test.jpg", bgr);
}

double computeFlowMagnitude(cv::Mat &flow, bool normalize_full) {
    cv::Mat flow_parts[2];
    cv::split(flow, flow_parts);
    cv::Mat magnitude, angle, magn_norm;
    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    // angle *= ((1.f / 360.f) * (180.f / 255.f));

    double sum_of_magnitude = cv::sum(magnitude)[0];
    if (true == normalize_full)
        sum_of_magnitude /= cv::countNonZero(magnitude);

    return sum_of_magnitude;
}
   
} // namespace cch


#endif