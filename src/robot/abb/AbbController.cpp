#include "AbbController.hpp"

// Nagłówki z vendor-dropa:
#include "abb_comm/abb_client.h"
#include "abb_comm/abb_messages.h"
// ^ ścieżki dopasuj do tego, jak skopiowałeś katalog abb_comm

#include <memory>
#include <chrono>
#include <thread>

namespace cymbergaj::robot::abb {

struct AbbController::Impl {
    std::string ip;
    uint16_t port;
    bool connected{false};
    std::unique_ptr<abb::comm::AbbClient> client; // klasa klienta z abb_comm
    // cache ustawień
    Speed speed{250, 180}; // domyślne
    Zone  zone{0.0};
};

AbbController::AbbController(std::string ip_robot, uint16_t port)
: impl_(new Impl{std::move(ip_robot), port}) {}

AbbController::~AbbController() {
    disconnect();
    delete impl_;
}

bool AbbController::connect() {
    if (impl_->connected) return true;
    impl_->client = std::make_unique<abb::comm::AbbClient>(impl_->ip, impl_->port);
    impl_->connected = impl_->client->connect();
    if (impl_->connected) {
        setSpeed(impl_->speed);
        setZone(impl_->zone);
    }
    return impl_->connected;
}

void AbbController::disconnect() {
    if (impl_->client) {
        impl_->client->disconnect();
        impl_->client.reset();
    }
    impl_->connected = false;
}

bool AbbController::isConnected() const { return impl_->connected; }

bool AbbController::setSpeed(const Speed& s) {
    if (!impl_->connected) return false;
    abb::comm::SetSpeed msg;
    msg.tcp_mm_s = s.tcp;
    msg.ori_deg_s = s.ori;
    if (impl_->client->send(msg)) { impl_->speed = s; return true; }
    return false;
}

bool AbbController::setZone(const Zone& z) {
    if (!impl_->connected) return false;
    abb::comm::SetZone msg;
    msg.fine_mm = z.fine_mm;            // uproszczenie; w RAPID będzie fine/approx
    if (impl_->client->send(msg)) { impl_->zone = z; return true; }
    return false;
}

bool AbbController::moveJ(const JointTarget& JT, bool wait) {
    if (!impl_->connected) return false;
    abb::comm::MoveJ m;
    for (int i=0;i<6;i++) m.j[i] = JT.j[i];  // [rad]
    if (!impl_->client->send(m)) return false;
    if (wait) impl_->client->waitDone();
    return true;
}

bool AbbController::moveL(const Pose& P, bool wait) {
    if (!impl_->connected) return false;
    abb::comm::MoveL m;
    m.x = P.p[0]; m.y = P.p[1]; m.z = P.p[2]; // [mm]
    m.qw = P.q[0]; m.qx = P.q[1]; m.qy = P.q[2]; m.qz = P.q[3];
    if (!impl_->client->send(m)) return false;
    if (wait) impl_->client->waitDone();
    return true;
}

std::optional<AbbController::JointTarget> AbbController::getJoints() {
    if (!impl_->connected) return std::nullopt;
    abb::comm::Joints state;
    if (!impl_->client->recvJoints(state)) return std::nullopt;
    JointTarget jt{};
    for (int i=0;i<6;i++) jt.j[i] = state.j[i];
    return jt;
}

std::optional<AbbController::Pose> AbbController::getPose() {
    if (!impl_->connected) return std::nullopt;
    abb::comm::Cartesian pose;
    if (!impl_->client->recvPose(pose)) return std::nullopt;
    Pose P{{pose.x,pose.y,pose.z},{pose.qw,pose.qx,pose.qy,pose.qz}};
    return P;
}

} // namespace
