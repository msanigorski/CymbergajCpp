// tracker.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <deque>

class KalmanTracker {
public:
    explicit KalmanTracker(const YAML::Node& config);
    
    void update(const cv::Point2f& measurement, uint64_t timestamp);
    void predictOnly(uint64_t timestamp);
    
    cv::Point2f getPosition() const;
    cv::Point2f getVelocity() const;
    std::vector<cv::Point2f> predictTrajectory(float horizon_seconds);
    
    void reset();
    
private:
    cv::KalmanFilter kf_;
    cv::Mat state_;
    cv::Mat measurement_;
    
    uint64_t last_timestamp_;
    bool initialized_;
    int frames_since_measurement_;
    
    // Table dimensions for bounce prediction
    cv::Rect2f table_bounds_;
    float restitution_coefficient_;
    
    // History for smoothing
    std::deque<cv::Point2f> position_history_;
    static const int HISTORY_SIZE = 5;
    
    void initKalman();
    cv::Point2f predictBounce(cv::Point2f pos, cv::Point2f vel, float dt);
    std::vector<cv::Point2f> simulateTrajectory(cv::Point2f start_pos, 
                                                cv::Point2f velocity, 
                                                float time_horizon,
                                                float dt = 0.01f);
};

