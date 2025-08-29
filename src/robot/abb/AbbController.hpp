#pragma once
#include <string>
#include <array>
#include <atomic>
#include <cstdint>

namespace cymbergaj { namespace robot { namespace abb {

/**
 * Minimal ABB controller (no ROS), uses protocol from abb_comm.*
 * - Connects via TCP to "motion" server on controller (IP/port).
 * - Sends commands formatted by abb_comm and waits for response.
 *
 * Note about quaternion:
 *  open_abb (abb_comm) expects order (q0, qx, qy, qz) where q0 = w.
 *  In moveL we accept std::array<double,7> in order:
 *     [x, y, z, q0(w), qx, qy, qz]  — exactly like abb_comm::setCartesian.
 */
class AbbController {
public:
    AbbController(std::string ip, uint16_t port);
    ~AbbController();

    bool connect();
    void disconnect();
    bool isConnected() const { return connected_; }

    // Configuration
    bool setWorkObject(const std::array<double,7>& wobj_xyz_q); // [x,y,z,q0,qx,qy,qz]
    bool setTool(const std::array<double,7>& tool_xyz_q);       // [x,y,z,q0,qx,qy,qz]
    bool setSpeed(double tcp_mm_s, double ori_deg_s);           // mm/s and deg/s
    bool setZoneFine();                                         // fine = true
    bool setZone(double p_tcp_mm, double p_ori_mm, double ori_deg); // fine=false

    // Movement
    bool moveL(const std::array<double,7>& pose_xyz_q); // [x,y,z,q0,qx,qy,qz]

    // Queries (optional)
    bool getCartesian(std::array<double,7>& pose_xyz_q);        // [x,y,z,q0,qx,qy,qz]
    bool getJoints(std::array<double,6>& joints_deg);           // degrees (parseJoints)

    const std::string& lastError() const { return last_error_; }

private:
    std::string ip_;
    uint16_t port_{0};
    int sock_{-1};
    std::atomic<bool> connected_{false};
    std::string last_error_;

    // TCP input/output with timeout (blocking with select()).
    // Returns true if response is semantically OK (code=1) and idCode matches (if >-1).
    bool sendAndReceive(const std::string& cmd, std::string& reply, int idCode = -1, int timeout_ms = 1000);

    // Helpers
    bool ensureConnected();
};

}}} // namespace