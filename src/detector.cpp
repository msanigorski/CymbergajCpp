// detector.cpp
#include "detector.hpp"
#include <algorithm>

Detector::Detector(const YAML::Node& config) 
    : method_(DetectionMethod::COLOR_THRESHOLD) {
    
    // Load color thresholds
    if (config["detection"]["hsv_lower"]) {
        auto lower = config["detection"]["hsv_lower"].as<std::vector<int>>();
        hsv_lower_ = cv::Scalar(lower[0], lower[1], lower[2]);
    } else {
        hsv_lower_ = cv::Scalar(0, 100, 100);  // Default red lower
    }
    
    if (config["detection"]["hsv_upper"]) {
        auto upper = config["detection"]["hsv_upper"].as<std::vector<int>>();
        hsv_upper_ = cv::Scalar(upper[0], upper[1], upper[2]);
    } else {
        hsv_upper_ = cv::Scalar(10, 255, 255);  // Default red upper
    }
    
    // Load contour validation parameters
    min_area_ = config["detection"]["min_area"].as<double>(100.0);
    max_area_ = config["detection"]["max_area"].as<double>(5000.0);
    min_circularity_ = config["detection"]["min_circularity"].as<double>(0.7);
    
    // Load ROI
    if (config["detection"]["roi"]) {
        auto roi = config["detection"]["roi"].as<std::vector<int>>();
        roi_ = cv::Rect(roi[0], roi[1], roi[2], roi[3]);
    } else {
        roi_ = cv::Rect(0, 0, 640, 480);
    }
    
    // Initialize background subtractor
    bg_subtractor_ = cv::createBackgroundSubtractorMOG2(500, 16, true);
    bg_subtractor_->setVarThreshold(10);
    
    // Pre-allocate matrices
    hsv_ = cv::Mat(480, 640, CV_8UC3);
    mask_ = cv::Mat(480, 640, CV_8UC1);
    fg_mask_ = cv::Mat(480, 640, CV_8UC1);
    
    // Morphological kernel for noise removal
    morph_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
}

bool Detector::detect(const cv::Mat& frame, cv::Point2f& puck_position) {
    if (method_ == DetectionMethod::COLOR_THRESHOLD) {
        return detectColor(frame, puck_position);
    } else {
        return detectBackgroundSub(frame, puck_position);
    }
}

bool Detector::detectColor(const cv::Mat& frame, cv::Point2f& position) {
    // Work on ROI
    cv::Mat roi_frame = frame(roi_);
    
    // Convert to HSV (reuse pre-allocated matrix)
    cv::cvtColor(roi_frame, hsv_, cv::COLOR_BGR2HSV);
    
    // Threshold for puck color
    cv::inRange(hsv_, hsv_lower_, hsv_upper_, mask_);
    
    // Morphological operations to remove noise
    cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, morph_kernel_);
    cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, morph_kernel_);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Find best contour
    double max_score = 0;
    cv::Point2f best_center;
    bool found = false;
    
    for (const auto& contour : contours) {
        if (!validateContour(contour)) continue;
        
        double area = cv::contourArea(contour);
        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        
        // Score based on circularity and area
        double score = circularity * std::min(area / min_area_, max_area_ / area);
        
        if (score > max_score) {
            max_score = score;
            best_center = getContourCenter(contour);
            found = true;
        }
    }
    
    if (found) {
        // Convert back to full frame coordinates
        position.x = best_center.x + roi_.x;
        position.y = best_center.y + roi_.y;
        return true;
    }
    
    return false;
}

bool Detector::detectBackgroundSub(const cv::Mat& frame, cv::Point2f& position) {
    // Work on ROI
    cv::Mat roi_frame = frame(roi_);
    
    // Update background model
    bg_subtractor_->apply(roi_frame, fg_mask_, 0.01);
    
    // Morphological operations
    cv::morphologyEx(fg_mask_, fg_mask_, cv::MORPH_OPEN, morph_kernel_);
    cv::dilate(fg_mask_, fg_mask_, morph_kernel_);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Find largest moving object (assumed to be puck)
    double max_area = 0;
    cv::Point2f best_center;
    bool found = false;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        if (area > min_area_ && area < max_area_) {
            // Check velocity constraint (optional)
            cv::Point2f center = getContourCenter(contour);
            
            if (area > max_area) {
                max_area = area;
                best_center = center;
                found = true;
            }
        }
    }
    
    if (found) {
        position.x = best_center.x + roi_.x;
        position.y = best_center.y + roi_.y;
        return true;
    }
    
    return false;
}

bool Detector::validateContour(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    
    if (area < min_area_ || area > max_area_) {
        return false;
    }
    
    double perimeter = cv::arcLength(contour, true);
    double circularity = 4 * CV_PI * area / (perimeter * perimeter);
    
    return circularity >= min_circularity_;
}

cv::Point2f Detector::getContourCenter(const std::vector<cv::Point>& contour) {
    cv::Moments m = cv::moments(contour);
    return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
}