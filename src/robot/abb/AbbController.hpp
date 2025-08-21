#pragma once
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <cstdint>

namespace cymbergaj::robot::abb {

// Prosty wrapper na open_abb (abb_comm)
class AbbController {
public:
    struct JointTarget { std::array<double,6> j; };        // [rad]
    struct Pose {                                          // TCP pose w ukł. robota
        std::array<double,3> p;                            // [mm] x,y,z
        std::array<double,4> q;                            // quaternion w,x,y,z
    };

    struct Speed { double tcp; double ori; };              // [mm/s], [deg/s]
    struct Zone  { double fine_mm; };                      // uproszczona "strefa" (fine/approx)

    // ip_robot: zwykle 192.168.125.1; port zgodny z SERVER.mod (domyślnie 11000)
    explicit AbbController(std::string ip_robot="192.168.125.1", uint16_t port=11000);
    ~AbbController();

    bool connect();
    void disconnect();
    bool isConnected() const;

    // Ustawienia ruchu
    bool setSpeed(const Speed& s);
    bool setZone(const Zone& z);

    bool moveL(const std::array<double,7>& pose);  
    // [x,y,z,qx,qy,qz,qw] w mm i kwaternion
    bool setWorkObject(const std::string& wobj);
    bool setTool(const std::string& tool);

    
    // Pomocnicze
    std::optional<JointTarget> getJoints();   // wymaga LOGGER po stronie RAPID
    std::optional<Pose>        getPose();     // jw.

private:
    struct Impl;               // PImpl — trzyma sockety/obiekty z abb_comm
    Impl* impl_;
};

} // namespace
