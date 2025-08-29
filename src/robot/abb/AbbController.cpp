#include "AbbController.hpp"
#include "abb_comm.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <sstream>

namespace cymbergaj { namespace robot { namespace abb {

AbbController::AbbController(std::string ip, uint16_t port)
    : ip_(std::move(ip)), port_(port) {}

AbbController::~AbbController() { disconnect(); }

bool AbbController::connect() {
    if (connected_) return true;
    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        last_error_ = "socket";
        return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) <= 0) {
        last_error_ = "bad ip";
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    if (::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        last_error_ = std::strerror(errno);
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    connected_ = true;
    return true;
}

void AbbController::disconnect() {
    if (sock_ >= 0) {
        std::string reply;
        std::string cmd = abb_comm::closeConnection(999);
        sendAndReceive(cmd, reply, 999);
        ::close(sock_);
    }
    sock_ = -1;
    connected_ = false;
}

bool AbbController::ensureConnected() {
    if (!connected_) return connect();
    return true;
}

bool AbbController::sendAndReceive(const std::string& cmd, std::string& reply, int idCode, int timeout_ms) {
    if (sock_ < 0) return false;
    ssize_t sent = ::send(sock_, cmd.c_str(), cmd.size(), 0);
    if (sent != static_cast<ssize_t>(cmd.size())) {
        last_error_ = "send";
        return false;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock_, &rfds);
    timeval tv{timeout_ms/1000, (timeout_ms%1000)*1000};
    int rv = ::select(sock_+1, &rfds, nullptr, nullptr, &tv);
    if (rv <= 0) {
        last_error_ = "timeout";
        return false;
    }
    char buf[1024];
    ssize_t len = ::recv(sock_, buf, sizeof(buf)-1, 0);
    if (len <= 0) {
        last_error_ = "recv";
        return false;
    }
    buf[len] = 0;
    reply.assign(buf, len);

    std::istringstream iss(reply);
    int instr=0, ok=0, rid=0;
    iss >> instr >> ok;
    if (idCode >= 0) iss >> rid;
    if (ok == 1 && (idCode < 0 || rid == idCode)) return true;

    last_error_ = reply;
    return false;
}

bool AbbController::setWorkObject(const std::array<double,7>& w) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::setWorkObject(w[0],w[1],w[2],w[3],w[4],w[5],w[6],10);
    std::string rep;
    return sendAndReceive(cmd, rep, 10);
}

bool AbbController::setTool(const std::array<double,7>& t) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::setTool(t[0],t[1],t[2],t[3],t[4],t[5],t[6],11);
    std::string rep;
    return sendAndReceive(cmd, rep, 11);
}

bool AbbController::setSpeed(double tcp_mm_s, double ori_deg_s) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::setSpeed(tcp_mm_s, ori_deg_s, 12);
    std::string rep;
    return sendAndReceive(cmd, rep, 12);
}

bool AbbController::setZoneFine() {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::setZone(true, 0, 0, 0, 13);
    std::string rep;
    return sendAndReceive(cmd, rep, 13);
}

bool AbbController::setZone(double p_tcp_mm, double p_ori_mm, double ori_deg) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::setZone(false, p_tcp_mm, p_ori_mm, ori_deg, 14);
    std::string rep;
    return sendAndReceive(cmd, rep, 14);
}

bool AbbController::moveL(const std::array<double,7>& pose_xyz_q) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::setCartesian(pose_xyz_q[0], pose_xyz_q[1], pose_xyz_q[2],
                                            pose_xyz_q[3], pose_xyz_q[4], pose_xyz_q[5], pose_xyz_q[6], 20);
    std::string rep;
    return sendAndReceive(cmd, rep, 20);
}

bool AbbController::getCartesian(std::array<double,7>& pose_xyz_q) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::getCartesian(30);
    std::string rep;
    if (!sendAndReceive(cmd, rep, 30)) return false;
    double x,y,z,q0,qx,qy,qz;
    if (abb_comm::parseCartesian(rep, &x,&y,&z,&q0,&qx,&qy,&qz) == 1) {
        pose_xyz_q = {x,y,z,q0,qx,qy,qz};
        return true;
    }
    return false;
}

bool AbbController::getJoints(std::array<double,6>& joints_deg) {
    if (!ensureConnected()) return false;
    std::string cmd = abb_comm::getJoints(31);
    std::string rep;
    if (!sendAndReceive(cmd, rep, 31)) return false;
    double j1,j2,j3,j4,j5,j6;
    if (abb_comm::parseJoints(rep, &j1,&j2,&j3,&j4,&j5,&j6) == 1) {
        joints_deg = {j1,j2,j3,j4,j5,j6};
        return true;
    }
    return false;
}

}}} // namespace

