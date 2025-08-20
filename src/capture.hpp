// capture.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <atomic>

class FrameCapture {
public:
    FrameCapture(const std::string& backend, int width, int height, int fps);
    ~FrameCapture();
    
    bool initialize();
    bool grab(cv::Mat& frame);
    void setPipeline(const std::string& pipeline);
    void release();
    
private:
    std::string backend_;
    std::string pipeline_;
    int width_;
    int height_;
    int fps_;
    
    cv::VideoCapture cap_;
    cv::Mat buffer_[2];  // Double buffering
    std::atomic<int> current_buffer_{0};
    
    bool initializeGStreamer();
    bool initializeLibcamera();
    void configureCameraSettings();
};