#include "RobotStrikePlanner.hpp"

#include <algorithm>
#include <array>
#include <iostream>

namespace cymbergaj::robot::abb {

RobotStrikePlanner::RobotStrikePlanner(AbbController& ctrl,
                                       double table_width_mm,
                                       double table_height_mm,
                                       double strike_height_mm)
    : controller(ctrl),
      table_width(table_width_mm),
      table_height(table_height_mm),
      strike_height(strike_height_mm) {}

void RobotStrikePlanner::strikeAt(const cv::Point2f& puck_pos) {
    if (!controller.isConnected()) {
        std::cerr << "[Planner] Robot not connected!" << std::endl;
        return;
    }

    double x = std::clamp<double>(puck_pos.x, 0.0, table_width);
    double y = std::clamp<double>(puck_pos.y, 0.0, table_height);
    double z = strike_height;

    std::array<double,7> pose{x, y, z, 1.0, 0.0, 0.0, 0.0};

    std::cout << "[Planner] Striking at puck (" << x << ", " << y << ")" << std::endl;
    controller.moveL(pose);
}

} // namespace cymbergaj::robot::abb

