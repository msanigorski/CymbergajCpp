// overlay.cpp
#include "overlay.hpp"
#include <sstream>
#include <iomanip>

Overlay::Overlay(const YAML::Node& config) {
    // Load colors
    if (config["overlay"]["colors"]) {
        auto colors = config["overlay"]["colors"];
        
        auto loadColor = [](const YAML::Node& node, const cv::Scalar& default_val) {
            if (node) {
                auto c = node.as<std::vector<int>>();
                return cv::Scalar(c[0], c[1], c[2]);
            }
            return default_val;
        };
        
        puck_color_ = loadColor(colors["puck"], cv::Scalar(0, 255, 0));
        velocity_color_ = loadColor(colors["velocity"], cv::Scalar(255, 0, 0));
        trajectory_color_ = loadColor(colors["trajectory"], cv::Scalar(255, 255, 0));
        bounce_color_ = loadColor(colors["bounce"], cv::Scalar(0, 255, 255));
        text_color_ = loadColor(colors["text"], cv::Scalar(255, 255, 255));
        goal_color_ = loadColor(colors["goal"], cv::Scalar(0, 0, 255));
    } else {
        // Default colors
        puck_color_ = cv::Scalar(0, 255, 0);      // Green
        velocity_color_ = cv::Scalar(255, 0, 0);   // Blue
        trajectory_color_ = cv::Scalar(255, 255, 0); // Cyan
        bounce_color_ = cv::Scalar(0, 255, 255);   // Yellow
        text_color_ = cv::Scalar(255, 255, 255);   // White
        goal_color_ = cv::Scalar(0, 0, 255);       // Red
    }
    
    // Load parameters
    puck_radius_ = config["overlay"]["puck_radius"].as<int>(10);
    velocity_scale_ = config["overlay"]["velocity_scale"].as<float>(10.0f);
    trajectory_points_ = config["overlay"]["trajectory_points"].as<int>(100);
    
    // Goal zones (example positions)
    goal_left_ = cv::Rect(0, 180, 20, 120);
    goal_right_ = cv::Rect(620, 180, 20, 120);
}

void Overlay::draw(cv::Mat& frame,
                  const cv::Point2f& position,
                  const cv::Point2f& velocity,
                  const std::vector<cv::Point2f>& trajectory,
                  bool detected) {
    
    // Draw goal zones
    drawGoalZones(frame);
    
    // Draw trajectory prediction
    if (!trajectory.empty()) {
        drawTrajectory(frame, trajectory);
    }
    
    // Draw puck
    if (detected || (position.x > 0 && position.y > 0)) {
        drawPuck(frame, position, detected);
        
        // Draw velocity vector
        if (std::abs(velocity.x) > 0.1 || std::abs(velocity.y) > 0.1) {
            drawVelocity(frame, position, velocity);
        }
    }
}

void Overlay::drawPuck(cv::Mat& frame, const cv::Point2f& pos, bool detected) {
    cv::Scalar color = detected ? puck_color_ : cv::Scalar(128, 128, 128);
    int thickness = detected ? 2 : 1;
    
    cv::circle(frame, cv::Point(pos), puck_radius_, color, thickness);
    
    // Draw center cross
    cv::drawMarker(frame, cv::Point(pos), color, cv::MARKER_CROSS, 5, 1);
}

void Overlay::drawVelocity(cv::Mat& frame, const cv::Point2f& pos, const cv::Point2f& vel) {
    cv::Point2f end_point = pos + vel * velocity_scale_;
    
    // Draw arrow
    cv::arrowedLine(frame, cv::Point(pos), cv::Point(end_point),
                   velocity_color_, 2, cv::LINE_AA, 0, 0.3);
    
    // Draw speed value
    float speed = cv::norm(vel);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << speed << " px/s";
    
    cv::Point text_pos(pos.x + 15, pos.y - 15);
    cv::putText(frame, ss.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
               0.4, velocity_color_, 1, cv::LINE_AA);
}

void Overlay::drawTrajectory(cv::Mat& frame, const std::vector<cv::Point2f>& points) {
    if (points.size() < 2) return;
    
    // Draw trajectory as dotted line
    for (size_t i = 1; i < points.size(); i++) {
        // Check for bounce markers (consecutive identical points)
        bool is_bounce = (i > 0 && points[i] == points[i-1]);
        
        if (is_bounce) {
            // Draw bounce point
            cv::circle(frame, cv::Point(points[i]), 5, bounce_color_, -1);
        } else if (i % 3 == 0) {  // Draw every 3rd segment for dotted effect
            cv::line(frame, cv::Point(points[i-1]), cv::Point(points[i]),
                    trajectory_color_, 1, cv::LINE_AA);
        }
    }
    
    // Draw predicted end point
    if (!points.empty()) {
        cv::circle(frame, cv::Point(points.back()), 8, trajectory_color_, 2);
    }
}

void Overlay::drawGoalZones(cv::Mat& frame) {
    // Draw semi-transparent goal zones
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay, goal_left_, goal_color_, -1);
    cv::rectangle(overlay, goal_right_, goal_color_, -1);
    cv::addWeighted(overlay, 0.3, frame, 0.7, 0, frame);
    
    // Draw goal zone borders
    cv::rectangle(frame, goal_left_, goal_color_, 2);
    cv::rectangle(frame, goal_right_, goal_color_, 2);
}

void Overlay::drawStats(cv::Mat& frame,
                       float fps,
                       float latency_ms,
                       int dropped_frames,
                       int detection_method) {
    
    // Background for text
    cv::Rect bg_rect(5, 5, 200, 80);
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay, bg_rect, cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.5, frame, 0.5, 0, frame);
    
    // Draw stats text
    int y_offset = 20;
    cv::Point text_pos(10, y_offset);
    
    std::stringstream ss;
    ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
    cv::putText(frame, ss.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
               0.5, text_color_, 1, cv::LINE_AA);
    
    text_pos.y += 20;
    ss.str("");
    ss << "Latency: " << std::fixed << std::setprecision(1) << latency_ms << " ms";
    cv::putText(frame, ss.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
               0.5, text_color_, 1, cv::LINE_AA);
    
    text_pos.y += 20;
    ss.str("");
    ss << "Drops: " << dropped_frames;
    cv::putText(frame, ss.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
               0.5, text_color_, 1, cv::LINE_AA);
    
    text_pos.y += 20;
    std::string method_str = (detection_method == 0) ? "Color" : "BgSub";
    ss.str("");
    ss << "Method: " << method_str;
    cv::putText(frame, ss.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
               0.5, text_color_, 1, cv::LINE_AA);
}