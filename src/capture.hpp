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

// capture.cpp
#include "capture.hpp"
#include <iostream>
#include <sstream>

FrameCapture::FrameCapture(const std::string& backend, int width, int height, int fps)
    : backend_(backend), width_(width), height_(height), fps_(fps) {
    // Pre-allocate buffers
    buffer_[0] = cv::Mat(height, width, CV_8UC3);
    buffer_[1] = cv::Mat(height, width, CV_8UC3);
}

FrameCapture::~FrameCapture() {
    release();
}

bool FrameCapture::initialize() {
    if (backend_ == "gstreamer") {
        return initializeGStreamer();
    } else if (backend_ == "libcamera") {
        return initializeLibcamera();
    } else {
        std::cerr << "Unknown backend: " << backend_ << std::endl;
        return false;
    }
}

bool FrameCapture::initializeGStreamer() {
    if (pipeline_.empty()) {
        // Default optimized pipeline for RPi Camera v2
        std::stringstream ss;
        ss << "libcamerasrc "
           << "! video/x-raw,width=" << width_ 
           << ",height=" << height_ 
           << ",framerate=" << fps_ << "/1,format=BGR "
           << "! videoconvert "
           << "! video/x-raw,format=BGR "
           << "! appsink drop=true max-buffers=2";
        pipeline_ = ss.str();
    }
    
    std::cout << "GStreamer pipeline: " << pipeline_ << std::endl;
    
    cap_.open(pipeline_, cv::CAP_GSTREAMER);
    
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open GStreamer pipeline" << std::endl;
        return false;
    }
    
    configureCameraSettings();
    return true;
}

bool FrameCapture::initializeLibcamera() {
    #ifdef HAS_LIBCAMERA
    // Direct libcamera implementation
    // This is a simplified version - full implementation would use libcamera C++ API
    std::stringstream ss;
    ss << "libcamera-vid -t 0 --inline --nopreview "
       << "--width " << width_ << " --height " << height_ 
       << " --framerate " << fps_ 
       << " --codec yuv420 --output - "
       << "| ffmpeg -f rawvideo -pix_fmt yuv420p -s:v " 
       << width_ << "x" << height_ << " -r " << fps_ 
       << " -i - -f v4l2 -pix_fmt bgr24 /dev/video0";
    
    // For simplicity, fall back to v4l2 with libcamera
    cap_.open(0, cv::CAP_V4L2);
    
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open libcamera device" << std::endl;
        return false;
    }
    
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('B','G','R','3'));
    
    configureCameraSettings();
    return true;
    #else
    std::cerr << "libcamera support not compiled in" << std::endl;
    return false;
    #endif
}

void FrameCapture::configureCameraSettings() {
    // Disable auto exposure for consistent performance
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);  // Manual mode
    cap_.set(cv::CAP_PROP_EXPOSURE, 10);  // Short exposure (1/1000s approx)
    
    // Fixed gain
    cap_.set(cv::CAP_PROP_GAIN, 1.0);
    
    // Disable auto white balance
    cap_.set(cv::CAP_PROP_AUTO_WB, 0);
    cap_.set(cv::CAP_PROP_WB_TEMPERATURE, 5000);
    
    // Set buffer size to minimum for low latency
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
}

bool FrameCapture::grab(cv::Mat& frame) {
    int buf_idx = current_buffer_.load();
    int next_idx = 1 - buf_idx;
    
    // Grab directly into pre-allocated buffer
    if (cap_.read(buffer_[next_idx])) {
        current_buffer_.store(next_idx);
        frame = buffer_[next_idx];  // Shallow copy
        return true;
    }
    return false;
}

void FrameCapture::setPipeline(const std::string& pipeline) {
    pipeline_ = pipeline;
}

void FrameCapture::release() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}