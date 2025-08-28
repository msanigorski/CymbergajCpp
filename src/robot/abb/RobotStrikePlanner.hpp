#pragma once

#include <opencv2/core.hpp>

#include "AbbController.hpp"

namespace cymbergaj::robot::abb {

// Translates puck coordinates to robot commands for air-hockey strikes.
class RobotStrikePlanner {
public:
    RobotStrikePlanner(AbbController& ctrl,
                       double table_width_mm = 800.0,
                       double table_height_mm = 400.0,
                       double strike_height_mm = 50.0);

    void strikeAt(const cv::Point2f& puck_pos);

private:
    AbbController& controller;
    double table_width;
    double table_height;
    double strike_height;
};

} // namespace cymbergaj::robot::abb

