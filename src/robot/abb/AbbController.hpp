#pragma once
#include <string>
#include <array>
#include <atomic>
#include <cstdint>

namespace cymbergaj { namespace robot { namespace abb {

/**
 * Minimalny kontroler ABB (bez ROS), używa protokołu z abb_comm.*
 *  - Łączy się TCP z serwerem "motion" na kontrolerze (IP/port).
 *  - Wysyła komendy sformatowane przez abb_comm i czeka na odpowiedź.
 *
 * Uwaga o kwaternionie:
 *  open_abb (abb_comm) oczekuje porządku (q0, qx, qy, qz) gdzie q0 = w.
 *  W moveL przyjmujemy std::array<double,7> w porządku:
 *     [x, y, z, q0(w), qx, qy, qz]  — dokładnie jak abb_comm::setCartesian.
 */
class AbbController {
public:
    AbbController(std::string ip, uint16_t port);
    ~AbbController();

    bool connect();
    void disconnect();
    bool isConnected() const { return connected_; }

    // Konfiguracje
    bool setWorkObject(const std::array<double,7>& wobj_xyz_q); // [x,y,z,q0,qx,qy,qz]
    bool setTool(const std::array<double,7>& tool_xyz_q);       // [x,y,z,q0,qx,qy,qz]
    bool setSpeed(double tcp_mm_s, double ori_deg_s);           // mm/s i deg/s (jak w abb_comm)
    bool setZoneFine();                                         // fine = true
    bool setZone(double p_tcp_mm, double p_ori_mm, double ori_deg); // fine=false

    // Ruchy
    bool moveL(const std::array<double,7>& pose_xyz_q); // [x,y,z,q0,qx,qy,qz]

    // Odczyty (opcjonalne)
    bool getCartesian(std::array<double,7>& pose_xyz_q);        // [x,y,z,q0,qx,qy,qz]
    bool getJoints(std::array<double,6>& joints_deg);           // stopnie (tak zwraca parseJoints)

    const std::string& lastError() const { return last_error_; }

private:
    std::string ip_;
    uint16_t port_{0};
    int sock_{-1};
    std::atomic<bool> connected_{false};
    std::string last_error_;

    // Wejście/wyjście TCP z timeoutem (blokujące z select()).
    // Zwraca true, jeśli odpowiedź semantycznie OK (kod=1) i idCode się zgadza (jeśli >-1).
    bool sendAndReceive(const std::string& cmd, std::string& reply, int idCode = -1, int timeout_ms = 1000);

    // Pomocnicze
    bool ensureConnected();
};

}}} // namespace
