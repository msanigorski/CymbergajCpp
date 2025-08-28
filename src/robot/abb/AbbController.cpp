#include "abb_comm.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstring>

// Debug flag - set to false to disable debug output
static const bool DEBUG_LOGGER = false;

AbbRobotController::AbbRobotController(const string& ip, int motion_port, int logger_port)
    : motion_socket(-1), robot_ip(ip), motion_port(motion_port), logger_socket(-1), logger_port(logger_port), logger_running(false) {
}

AbbRobotController::~AbbRobotController() {
    disconnect();
}

bool AbbRobotController::connect() {
    std::cout << "Connecting to ABB robot at " << robot_ip << std::endl;
    
    // Connect to motion server first
    if (!connectToMotionServer()) {
        std::cerr << "Failed to connect to motion server" << std::endl;
        return false;
    }
    
    std::cout << "Connected to motion server (port " << motion_port << ")" << std::endl;
    
    // Try to connect to logger server (optional)
    if (connectToLoggerServer()) {
        std::cout << "Connected to logger server (port " << logger_port << ")" << std::endl;
        startPositionLogging();
    } else {
        std::cout << "Warning: Could not connect to logger server - position feedback disabled" << std::endl;
    }
    
    return true;
}

bool AbbRobotController::connectToMotionServer() {
    motion_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (motion_socket < 0) {
        std::cerr << "Failed to create motion socket" << std::endl;
        return false;
    }
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(motion_port);
    
    if (inet_pton(AF_INET, robot_ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid IP address: " << robot_ip << std::endl;
        close(motion_socket);
        motion_socket = -1;
        return false;
    }
    
    if (::connect(motion_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to connect to motion server at " << robot_ip << ":" << motion_port << std::endl;
        close(motion_socket);
        motion_socket = -1;
        return false;
    }
    
    return true;
}

bool AbbRobotController::connectToLoggerServer() {
    logger_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (logger_socket < 0) {
        std::cerr << "Failed to create logger socket" << std::endl;
        return false;
    }
    
    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 5;  // 5 second timeout
    timeout.tv_usec = 0;
    setsockopt(logger_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(logger_port);
    
    if (inet_pton(AF_INET, robot_ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid IP address for logger: " << robot_ip << std::endl;
        close(logger_socket);
        logger_socket = -1;
        return false;
    }
    
    if (::connect(logger_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to connect to logger server at " << robot_ip << ":" << logger_port << std::endl;
        close(logger_socket);
        logger_socket = -1;
        return false;
    }
    
    return true;
}

void AbbRobotController::disconnect() {
    stopPositionLogging();
    
    if (motion_socket >= 0) {
        // Send close connection command
        sendCommand(abb_comm::closeConnection(99));
        close(motion_socket);
        motion_socket = -1;
    }
    
    if (logger_socket >= 0) {
        close(logger_socket);
        logger_socket = -1;
    }
}

bool AbbRobotController::isConnected() const {
    return motion_socket >= 0;
}

bool AbbRobotController::sendCommand(const string& command) {
    if (motion_socket < 0) {
        std::cerr << "Not connected to motion server" << std::endl;
        return false;
    }
    
    ssize_t sent = send(motion_socket, command.c_str(), command.length(), 0);
    if (sent != (ssize_t)command.length()) {
        std::cerr << "Failed to send complete command" << std::endl;
        return false;
    }
    
    std::cout << "Sent: " << command << std::endl;
    return true;
}

string AbbRobotController::sendCommandWithResponse(const string& command) {
    if (!sendCommand(command)) {
        return "";
    }
    
    char buffer[4096];
    ssize_t received = recv(motion_socket, buffer, sizeof(buffer) - 1, 0);
    if (received > 0) {
        buffer[received] = '\0';
        string response(buffer);
        std::cout << "Sent: " << command;
        std::cout << "Received: " << response << std::endl;
        return response;
    }
    
    std::cout << "No response received!" << std::endl;
    return "";
}

bool AbbRobotController::startPositionLogging() {
    if (logger_socket < 0) {
        std::cerr << "Logger not connected" << std::endl;
        return false;
    }
    
    if (logger_running.load()) {
        return true;  // Already running
    }
    
    logger_running.store(true);
    logger_thread = std::thread(&AbbRobotController::loggerWorker, this);
    
    return true;
}

void AbbRobotController::stopPositionLogging() {
    if (logger_running.load()) {
        logger_running.store(false);
        if (logger_thread.joinable()) {
            logger_thread.join();
        }
    }
}

void AbbRobotController::loggerWorker() {
    char buffer[1024];
    string accumulated_data;
    
    std::cout << "Logger worker started" << std::endl;
    
    while (logger_running.load() && logger_socket >= 0) {
        ssize_t received = recv(logger_socket, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
        
        if (received > 0) {
            buffer[received] = '\0';
            accumulated_data += string(buffer);
            
            std::cout << "DEBUG LOGGER: Received " << received << " bytes: '" << string(buffer) << "'" << std::endl;
            
            // Process complete messages (look for message delimiters)
            size_t delimiter_pos;
            while ((delimiter_pos = accumulated_data.find('\n')) != string::npos) {
                string message = accumulated_data.substr(0, delimiter_pos);
                accumulated_data.erase(0, delimiter_pos + 1);
                
                std::cout << "DEBUG LOGGER: Processing message: '" << message << "'" << std::endl;
                
                // Parse the message and update position
                RobotPosition pos;
                if (parseLoggerMessage(message, pos)) {
                    std::lock_guard<std::mutex> lock(position_mutex);
                    current_position = pos;
                    current_position.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    
                    std::cout << "DEBUG LOGGER: Successfully parsed position: x=" << pos.x 
                              << " y=" << pos.y << " z=" << pos.z << std::endl;
                    
                    // Call callback if set
                    if (position_callback) {
                        position_callback(current_position);
                    }
                } else {
                    std::cout << "DEBUG LOGGER: Failed to parse message" << std::endl;
                }
            }
        } else if (received == 0) {
            std::cerr << "Logger connection closed by server" << std::endl;
            break;
        } else {
            // No data available or error, sleep briefly
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    std::cout << "Logger worker stopped" << std::endl;
}

bool AbbRobotController::parseLoggerMessage(const std::string& message, RobotPosition& pos) {
    // Oczekiwane formaty:
    // 1) Nowy:
    //    "# 0 YYYY-MM-DD HH:MM:SS secs  x y z qw qx qy qz"
    //    "# 1 YYYY-MM-DD HH:MM:SS secs  j1 j2 j3 j4 j5 j6"
    // 2) Legacy:
    //    "# timestamp x y z qw qx qy qz j1 j2 j3 j4 j5 j6 [eax_a ... eax_f]"

    auto isInt = [](const std::string& s)->bool {
        if (s.empty()) return false;
        char* end=nullptr;
        std::strtol(s.c_str(), &end, 10);
        return *end == '\0';
    };
    auto isNum = [](const std::string& s)->bool {
        if (s.empty()) return false;
        char* end=nullptr;
        std::strtod(s.c_str(), &end);
        return *end == '\0';
    };

    // Tokenizacja po białych znakach
    std::vector<std::string> t;
    {
        std::istringstream iss(message);
        std::string tok;
        while (iss >> tok) t.push_back(tok);
    }
    if (t.empty() || t[0] != "#") return false;

    size_t i = 1;
    // Wariant z indeksem strumienia?
    int stream = -1;
    if (i < t.size() && isInt(t[i])) {
        stream = std::stoi(t[i]);
        ++i;
    }

    // Wariant nowy: mamy datę i czas -> przeskocz 2 tokeny (YYYY-MM-DD, HH:MM:SS) + 1 token (sekundy ułamkowe)
    auto looksLikeDateTime = [&](size_t idx)->bool {
        if (idx + 2 >= t.size()) return false;
        // YYYY-MM-DD i HH:MM:SS (proste heurystyki)
        return (t[idx].find('-') != std::string::npos) && (t[idx+1].find(':') != std::string::npos) && isNum(t[idx+2]);
    };

    try {
        if (stream == 0 || stream == 1) {
            // Nowy format z dwoma strumieniami
            if (looksLikeDateTime(i)) i += 3; // pomin: data, czas, sekundy

            if (stream == 0) {
                // TCP: x y z qw qx qy qz
                if (i + 7 > t.size()) return false;
                pos.x  = std::stod(t[i+0]);
                pos.y  = std::stod(t[i+1]);
                pos.z  = std::stod(t[i+2]);
                pos.q0 = std::stod(t[i+3]);  // uwaga: qw
                pos.qx = std::stod(t[i+4]);
                pos.qy = std::stod(t[i+5]);
                pos.qz = std::stod(t[i+6]);
                pos.valid = true;
            } else { 
                // JOINTS: j1..j6
                if (i + 6 > t.size()) return false;
                pos.joint1 = std::stod(t[i+0]);
                pos.joint2 = std::stod(t[i+1]);
                pos.joint3 = std::stod(t[i+2]);
                pos.joint4 = std::stod(t[i+3]);
                pos.joint5 = std::stod(t[i+4]);
                pos.joint6 = std::stod(t[i+5]);
                // nie zmieniamy pos.valid jeżeli wcześniej było true,
                // ale jeżeli nic jeszcze nie było – ustawiamy:
                if (!pos.valid) pos.valid = true;
            }
        } else {
            // Legacy: bez indeksu strumienia; próbujemy zczytać kolejno:
            // timestamp (num), x y z qw qx qy qz [j1..j6] [eax_a..eax_f]
            if (i >= t.size() || !isNum(t[i])) return false; // timestamp
            ++i;

            if (i + 7 > t.size()) return false;
            pos.x  = std::stod(t[i+0]);
            pos.y  = std::stod(t[i+1]);
            pos.z  = std::stod(t[i+2]);
            pos.q0 = std::stod(t[i+3]);
            pos.qx = std::stod(t[i+4]);
            pos.qy = std::stod(t[i+5]);
            pos.qz = std::stod(t[i+6]);
            pos.valid = true;
            i += 7;

            // Opcjonalne JOINTS
            if (i + 6 <= t.size()) {
                bool okJ = true;
                for (size_t k = 0; k < 6; ++k) okJ = okJ && isNum(t[i+k]);
                if (okJ) {
                    pos.joint1 = std::stod(t[i+0]);
                    pos.joint2 = std::stod(t[i+1]);
                    pos.joint3 = std::stod(t[i+2]);
                    pos.joint4 = std::stod(t[i+3]);
                    pos.joint5 = std::stod(t[i+4]);
                    pos.joint6 = std::stod(t[i+5]);
                    i += 6;
                }
            }
            // Opcjonalne zewn. osie (do 6 sztuk)
            double* eax[6] = {&pos.eax_a,&pos.eax_b,&pos.eax_c,&pos.eax_d,&pos.eax_e,&pos.eax_f};
            for (int k = 0; k < 6 && i < t.size(); ++k) {
                if (!isNum(t[i])) break;
                *eax[k] = std::stod(t[i]);
                ++i;
            }
        }

        pos.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
        return true;
    } catch (...) {
        return false;
    }
}

RobotPosition AbbRobotController::getCurrentPosition() {
    std::lock_guard<std::mutex> lock(position_mutex);
    return current_position;
}

void AbbRobotController::setPositionCallback(std::function<void(const RobotPosition&)> callback) {
    position_callback = callback;
}

// Convenience methods
bool AbbRobotController::moveCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode) {
    string command = abb_comm::setCartesian(x, y, z, q0, qx, qy, qz, idCode);
    return sendCommand(command);
}

bool AbbRobotController::moveJoints(double j1, double j2, double j3, double j4, double j5, double j6, int idCode) {
    string command = abb_comm::setJoints(j1, j2, j3, j4, j5, j6, idCode);
    return sendCommand(command);
}

bool AbbRobotController::setSpeed(double tcp, double ori, int idCode) {
    string command = abb_comm::setSpeed(tcp, ori, idCode);
    return sendCommand(command);
}

bool AbbRobotController::setVacuum(bool on, int idCode) {
    string command = abb_comm::setVacuum(on ? 1 : 0, idCode);
    return sendCommand(command);
}

bool AbbRobotController::ping(int idCode) {
    string command = abb_comm::pingRobot(idCode);
    string response = sendCommandWithResponse(command);
    return !response.empty();
}

bool AbbRobotController::queryCartesianPosition(double& x, double& y, double& z, double& q0, double& qx, double& qy, double& qz) {
    string command = abb_comm::getCartesian(0);
    string response = sendCommandWithResponse(command);
    
    if (response.empty()) {
        std::cout << "No response to getCartesian query" << std::endl;
        return false;
    }
    
    // Check if response looks valid (should contain more than just "3 0")
    if (response.length() < 20) {  // Rough check - real position data should be much longer
        std::cout << "Response too short for position data: '" << response << "'" << std::endl;
        return false;
    }
    
    int result = abb_comm::parseCartesian(response, &x, &y, &z, &q0, &qx, &qy, &qz);
    if (result >= 0) {
        std::cout << "Successfully parsed position: x=" << x << " y=" << y << " z=" << z << std::endl;
        return true;
    } else {
        std::cout << "Failed to parse cartesian response: '" << response << "'" << std::endl;
        return false;
    }
}

bool AbbRobotController::queryJointPositions(double& j1, double& j2, double& j3, double& j4, double& j5, double& j6) {
    string command = abb_comm::getJoints(0);
    string response = sendCommandWithResponse(command);
    
    if (response.empty()) {
        return false;
    }
    
    int result = abb_comm::parseJoints(response, &j1, &j2, &j3, &j4, &j5, &j6);
    return result >= 0;
}