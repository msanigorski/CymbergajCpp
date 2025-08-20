// overlay.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

class Overlay {
public:
    explicit Overlay(const YAML::Node& config);
    
    void draw(cv::Mat& frame, 
              const cv::Point2f& position,
              const cv::Point2f& velocity,
              const std::vector<cv::Point2f>& trajectory,
              bool detected);
    
    void drawStats(cv::Mat& frame,
                  float fps,
                  float latency_ms,
                  int dropped_frames,
                  int detection_method);
    
private:
    // Colors
    cv::Scalar puck_color_;
    cv::Scalar velocity_color_;
    cv::Scalar trajectory_color_;
    cv::Scalar bounce_color_;
    cv::Scalar text_color_;
    cv::Scalar goal_color_;
    
    // Visualization parameters
    int puck_radius_;
    float velocity_scale_;
    int trajectory_points_;
    
    // Goal zones
    cv::Rect goal_left_;
    cv::Rect goal_right_;
    
    void drawPuck(cv::Mat& frame, const cv::Point2f& pos, bool detected);
    void drawVelocity(cv::Mat& frame, const cv::Point2f& pos, const cv::Point2f& vel);
    void drawTrajectory(cv::Mat& frame, const std::vector<cv::Point2f>& points);
    void drawGoalZones(cv::Mat& frame);
};