// tracker.cpp
#include "tracker.hpp"
#include <cmath>

KalmanTracker::KalmanTracker(const YAML::Node& config) 
    : kf_(4, 2, 0), initialized_(false), frames_since_measurement_(0),
      last_timestamp_(0) {
    
    // Load table dimensions
    if (config["table"]["dimensions"]) {
        auto dims = config["table"]["dimensions"].as<std::vector<float>>();
        table_bounds_ = cv::Rect2f(0, 0, dims[0], dims[1]);
    } else {
        table_bounds_ = cv::Rect2f(0, 0, 640, 480);  // Default in pixels
    }
    
    // Load restitution coefficient
    restitution_coefficient_ = config["physics"]["restitution"].as<float>(0.9f);
    
    initKalman();
}

void KalmanTracker::initKalman() {
    // State: [x, y, vx, vy]
    // Measurement: [x, y]
    
    // State transition matrix (constant velocity model)
    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    
    // Measurement matrix
    kf_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);
    
    // Process noise covariance
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar(1e-4));
    kf_.processNoiseCov.at<float>(2, 2) = 1e-2;  // Higher noise for velocity
    kf_.processNoiseCov.at<float>(3, 3) = 1e-2;
    
    // Measurement noise covariance
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(1e-1));
    
    // Error covariance
    cv::setIdentity(kf_.errorCovPost, cv::Scalar(1));
    
    // Initial state
    state_ = cv::Mat::zeros(4, 1, CV_32F);
    measurement_ = cv::Mat::zeros(2, 1, CV_32F);
}

void KalmanTracker::update(const cv::Point2f& measurement, uint64_t timestamp) {
    float dt = 0.0111f;  // Default to 90 fps
    
    if (initialized_ && last_timestamp_ > 0) {
        dt = (timestamp - last_timestamp_) / 1000000.0f;  // Convert microseconds to seconds
    }
    
    // Update transition matrix with actual dt
    kf_.transitionMatrix.at<float>(0, 2) = dt;
    kf_.transitionMatrix.at<float>(1, 3) = dt;
    
    if (!initialized_) {
        // First measurement - initialize state
        state_.at<float>(0) = measurement.x;
        state_.at<float>(1) = measurement.y;
        state_.at<float>(2) = 0;
        state_.at<float>(3) = 0;
        kf_.statePost = state_.clone();
        initialized_ = true;
    } else {
        // Predict
        state_ = kf_.predict();
        
        // Update with measurement
        measurement_.at<float>(0) = measurement.x;
        measurement_.at<float>(1) = measurement.y;
        state_ = kf_.correct(measurement_);
    }
    
    // Update history
    position_history_.push_back(cv::Point2f(state_.at<float>(0), state_.at<float>(1)));
    if (position_history_.size() > HISTORY_SIZE) {
        position_history_.pop_front();
    }
    
    frames_since_measurement_ = 0;
    last_timestamp_ = timestamp;
}

void KalmanTracker::predictOnly(uint64_t timestamp) {
    if (!initialized_) return;
    
    float dt = (timestamp - last_timestamp_) / 1000000.0f;
    
    // Update transition matrix
    kf_.transitionMatrix.at<float>(0, 2) = dt;
    kf_.transitionMatrix.at<float>(1, 3) = dt;
    
    // Predict only
    state_ = kf_.predict();
    
    frames_since_measurement_++;
    last_timestamp_ = timestamp;
    
    // Reset if lost for too long
    if (frames_since_measurement_ > 30) {  // Lost for 1/3 second at 90fps
        reset();
    }
}

cv::Point2f KalmanTracker::getPosition() const {
    if (!initialized_) return cv::Point2f(0, 0);
    return cv::Point2f(state_.at<float>(0), state_.at<float>(1));
}

cv::Point2f KalmanTracker::getVelocity() const {
    if (!initialized_) return cv::Point2f(0, 0);
    return cv::Point2f(state_.at<float>(2), state_.at<float>(3));
}

std::vector<cv::Point2f> KalmanTracker::predictTrajectory(float horizon_seconds) {
    if (!initialized_) return std::vector<cv::Point2f>();
    
    cv::Point2f pos = getPosition();
    cv::Point2f vel = getVelocity();
    
    return simulateTrajectory(pos, vel, horizon_seconds);
}

std::vector<cv::Point2f> KalmanTracker::simulateTrajectory(cv::Point2f start_pos, 
                                                           cv::Point2f velocity,
                                                           float time_horizon,
                                                           float dt) {
    std::vector<cv::Point2f> trajectory;
    trajectory.reserve(static_cast<int>(time_horizon / dt));
    
    cv::Point2f pos = start_pos;
    cv::Point2f vel = velocity;
    
    for (float t = 0; t < time_horizon; t += dt) {
        // Update position
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        
        // Check for bounces
        bool bounced = false;
        
        // Left/right walls
        if (pos.x <= table_bounds_.x || pos.x >= table_bounds_.x + table_bounds_.width) {
            vel.x = -vel.x * restitution_coefficient_;
            pos.x = std::max(table_bounds_.x, 
                           std::min(pos.x, table_bounds_.x + table_bounds_.width));
            bounced = true;
        }
        
        // Top/bottom walls
        if (pos.y <= table_bounds_.y || pos.y >= table_bounds_.y + table_bounds_.height) {
            vel.y = -vel.y * restitution_coefficient_;
            pos.y = std::max(table_bounds_.y,
                           std::min(pos.y, table_bounds_.y + table_bounds_.height));
            bounced = true;
        }
        
        trajectory.push_back(pos);
        
        // Add bounce point marker
        if (bounced && !trajectory.empty()) {
            trajectory.push_back(pos);  // Double point to mark bounce
        }
    }
    
    return trajectory;
}

void KalmanTracker::reset() {
    initialized_ = false;
    frames_since_measurement_ = 0;
    position_history_.clear();
    initKalman();
}