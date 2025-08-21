void RobotStrikePlanner::strikeAt(const cv::Point2f& puck_pos) {
    if (!controller.isConnected()) {
        std::cerr << "[Planner] Robot not connected!" << std::endl;
        return;
    }

    // mapowanie stołu cymbergaja (np. 800x400 mm) na przestrzeń robota
    double table_width = 800.0;   // mm
    double table_height = 400.0;  // mm

    // załóżmy, że workobject w (0,0,0) to lewy-dół stołu
    double x = puck_pos.x;  // mm w układzie stołu
    double y = puck_pos.y;  // mm w układzie stołu
    double z = 50.0;        // wysokość TCP nad stołem

    // orientacja TCP: pionowo w dół
    std::array<double,7> pose = {x, y, z, 0, 0, 0, 1};  // kwaternion [0,0,0,1]

    std::cout << "[Planner] Striking at puck (" << puck_pos.x << "," << puck_pos.y << ")" << std::endl;
    controller.moveL(pose);
}
