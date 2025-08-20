// detector.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>

enum class DetectionMethod {
    COLOR_THRESHOLD = 0,
    BACKGROUND_SUB = 1
};

class Detector {
public:
    explicit Detector(const YAML::Node& config);
    
    bool detect(const cv::Mat& frame, cv::Point2f& puck_position);
    void setMethod(DetectionMethod method) { method_ = method; }
    DetectionMethod getMethod() const { return method_; }
    
private:
    DetectionMethod method_;
    
    // Color detection parameters
    cv::Scalar hsv_lower_;
    cv::Scalar hsv_upper_;
    
    // Background subtraction
    cv::Ptr<cv::BackgroundSubtractor> bg_subtractor_;
    
    // Common parameters
    double min_area_;
    double max_area_;
    double min_circularity_;
    cv::Rect roi_;
    
    // Pre-allocated matrices
    cv::Mat hsv_;
    cv::Mat mask_;
    cv::Mat morph_kernel_;
    cv::Mat fg_mask_;
    
    // Methods
    bool detectColor(const cv::Mat& frame, cv::Point2f& position);
    bool detectBackgroundSub(const cv::Mat& frame, cv::Point2f& position);
    bool validateContour(const std::vector<cv::Point>& contour);
    cv::Point2f getContourCenter(const std::vector<cv::Point>& contour);
};
