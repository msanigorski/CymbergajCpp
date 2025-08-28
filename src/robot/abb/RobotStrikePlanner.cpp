#include "robot/abb/RobotStrikePlanner.hpp"
#include "robot/abb/AbbController.hpp"

namespace cymbergaj { namespace robot { namespace abb {

using Pose7 = std::array<double,7>;

Pose7 RobotStrikePlanner::mapPuckToTCP(const cv::Point2f& p) {
    // PRZYKŁADOWY mapping (dopasuj do stołu i kalibracji!)
    // Załóżmy układ: 1 px = 1 mm, (0,0) w lewym górnym rogu obrazu.
    // Ustawiamy stałą wysokość Z i kwaternion „do dołu”.
    const double x_mm = static_cast<double>(p.x);
    const double y_mm = static_cast<double>(p.y);
    const double z_mm = 50.0;           // wysokość nad stołem (mm) — dopasuj
    const double q0 = 1.0, qx = 0.0, qy = 0.0, qz = 0.0; // brak rotacji (w,x,y,z)

    return { x_mm, y_mm, z_mm, q0, qx, qy, qz };
}

void RobotStrikePlanner::strikeAt(const cv::Point2f& puck_pos) {
    if (!ctrl_) return;

    // 1) Ustal wobj/tool/speed/zone — jednorazowo gdzie indziej;
    // tu tylko ruch. (Załóżmy, że już ustawione).
    // 2) Wylicz docelowy TCP
    Pose7 pose = mapPuckToTCP(puck_pos);

    // 3) MoveL
    (void)ctrl_->moveL(pose);
}

}}} // namespace
