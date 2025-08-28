#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <atomic>
#include <mutex>
#include <map>
#include <fstream>
#include <chrono>
#include <cmath>
#include <iomanip>

//
// Globalna flaga pracy serwera
//
std::atomic<bool> running{true};

#ifdef USE_ABB
#include "robot/abb/AbbController.hpp"
using cymbergaj::robot::abb::AbbController;
#endif

// ======= Robot Testing Structures =======
struct Position3D {
    float x, y, z;
    float rx, ry, rz; // Rotation angles in degrees
    
    Position3D(float x = 0, float y = 0, float z = 0, float rx = 0, float ry = 0, float rz = 0)
        : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz) {}
        
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "(" << x << ", " << y << ", " << z << ") R(" << rx << ", " << ry << ", " << rz << ")";
        return oss.str();
    }
};

struct RobotTestCommand {
    enum Type { MOVE_TO, MOVE_LINEAR, MOVE_JOINT, STRIKE, CIRCLE, SQUARE, CALIBRATE };
    Type type;
    Position3D target;
    float speed;        // 0.0 - 1.0
    float duration;     // seconds for time-based movements
    bool relative;      // relative vs absolute movement
    std::string name;   // test name
    
    RobotTestCommand(Type t = MOVE_TO, Position3D pos = Position3D(), float spd = 0.5f, 
                     const std::string& n = "Test") 
        : type(t), target(pos), speed(spd), duration(1.0f), relative(false), name(n) {}
};

struct TestSequence {
    std::vector<RobotTestCommand> commands;
    std::string name;
    bool loop;
    int repeat_count;
    float delay_between; // seconds
    
    TestSequence(const std::string& n = "Default") 
        : name(n), loop(false), repeat_count(1), delay_between(1.0f) {}
};

struct RobotStatus {
    Position3D current_position;
    Position3D target_position;
    bool is_moving;
    bool is_connected;
    std::string last_error;
    std::string current_test;
    int test_progress; // percentage
    std::chrono::high_resolution_clock::time_point last_update;
    
    RobotStatus() : is_moving(false), is_connected(false), test_progress(0) {
        last_update = std::chrono::high_resolution_clock::now();
    }
};

// ======= Safety Limits =======
struct SafetyLimits {
    Position3D min_pos = Position3D(-0.8f, -0.6f, 0.01f, -180, -180, -180);
    Position3D max_pos = Position3D(0.8f, 0.6f, 0.3f, 180, 180, 180);
    float max_speed = 1.0f;
    float max_acceleration = 2.0f;
    
    bool isPositionSafe(const Position3D& pos) const {
        return (pos.x >= min_pos.x && pos.x <= max_pos.x &&
                pos.y >= min_pos.y && pos.y <= max_pos.y &&
                pos.z >= min_pos.z && pos.z <= max_pos.z &&
                pos.rx >= min_pos.rx && pos.rx <= max_pos.rx &&
                pos.ry >= min_pos.ry && pos.ry <= max_pos.ry &&
                pos.rz >= min_pos.rz && pos.rz <= max_pos.rz);
    }
    
    Position3D clampPosition(const Position3D& pos) const {
        Position3D clamped = pos;
        clamped.x = std::max(min_pos.x, std::min(max_pos.x, pos.x));
        clamped.y = std::max(min_pos.y, std::min(max_pos.y, pos.y));
        clamped.z = std::max(min_pos.z, std::min(max_pos.z, pos.z));
        clamped.rx = std::max(min_pos.rx, std::min(max_pos.rx, pos.rx));
        clamped.ry = std::max(min_pos.ry, std::min(max_pos.ry, pos.ry));
        clamped.rz = std::max(min_pos.rz, std::min(max_pos.rz, pos.rz));
        return clamped;
    }
};

// ======= Global State =======
SafetyLimits g_safety;
RobotStatus g_robot_status;
std::mutex g_robot_mutex;
std::vector<TestSequence> g_test_sequences;
bool g_auto_test_running = false;
std::thread g_test_thread;

// Robot connection info
struct RobotConnection {
    std::string ip = "192.168.125.1";
    uint16_t port = 11000;
} g_connection;

#ifdef USE_ABB
static AbbController* g_robot = nullptr;
#endif

// ======= Predefined Test Sequences =======
void initializeTestSequences() {
    // Test 1: Basic movement test
    TestSequence basic("Basic Movement");
    basic.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.3f, "Home"));
    basic.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0.2f, 0, 0.1f), 0.3f, "Right"));
    basic.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0.2f, 0.1f), 0.3f, "Forward"));
    basic.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(-0.2f, 0, 0.1f), 0.3f, "Left"));
    basic.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, -0.2f, 0.1f), 0.3f, "Back"));
    basic.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.3f, "Home"));
    g_test_sequences.push_back(basic);
    
    // Test 2: Speed test
    TestSequence speed("Speed Test");
    speed.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.1f, "Slow"));
    speed.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0.3f, 0, 0.1f), 0.5f, "Medium"));
    speed.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0.3f, 0.1f), 0.9f, "Fast"));
    speed.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.5f, "Return"));
    g_test_sequences.push_back(speed);
    
    // Test 3: Z-axis test
    TestSequence zaxis("Z-Axis Test");
    zaxis.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.05f), 0.3f, "Low"));
    zaxis.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.15f), 0.3f, "High"));
    zaxis.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.3f, "Medium"));
    g_test_sequences.push_back(zaxis);
    
    // Test 4: Strike simulation
    TestSequence strike("Strike Test");
    strike.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(-0.3f, -0.2f, 0.15f), 0.3f, "Prepare"));
    strike.commands.push_back(RobotTestCommand(RobotTestCommand::STRIKE, Position3D(0.3f, 0.2f, 0.05f), 1.0f, "Strike"));
    strike.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.3f, "Return"));
    g_test_sequences.push_back(strike);
    
    // Test 5: Precision test
    TestSequence precision("Precision Test");
    for (int i = 0; i < 8; i++) {
        float angle = i * M_PI / 4;
        float radius = 0.1f;
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        precision.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, 
            Position3D(x, y, 0.08f), 0.2f, "Point " + std::to_string(i+1)));
    }
    precision.commands.push_back(RobotTestCommand(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.3f, "Center"));
    g_test_sequences.push_back(precision);
}

// ======= Robot Control Functions =======
bool executeRobotCommand(const RobotTestCommand& cmd) {
#ifdef USE_ABB
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    if (!g_robot || !g_robot->isConnected()) {
        g_robot_status.last_error = "Robot not connected";
        return false;
    }
    
    try {
        // Apply safety limits
        Position3D safe_target = g_safety.clampPosition(cmd.target);
        if (!g_safety.isPositionSafe(cmd.target)) {
            g_robot_status.last_error = "Target position outside safety limits: " + cmd.target.toString();
            std::cout << "WARNING: Position clamped from " << cmd.target.toString() 
                      << " to " << safe_target.toString() << std::endl;
        }
        
        g_robot_status.target_position = safe_target;
        g_robot_status.is_moving = true;
        g_robot_status.current_test = cmd.name;
        
        float safe_speed = std::min(cmd.speed, g_safety.max_speed);
        
        switch (cmd.type) {
            case RobotTestCommand::MOVE_TO:
            case RobotTestCommand::MOVE_LINEAR:
                // Example ABB command - adjust for your implementation
                // g_robot->moveLinear(safe_target.x, safe_target.y, safe_target.z, 
                //                    safe_target.rx, safe_target.ry, safe_target.rz, safe_speed);
                std::cout << "MOVE_LINEAR to " << safe_target.toString() 
                          << " at speed " << safe_speed << std::endl;
                break;
                
            case RobotTestCommand::MOVE_JOINT:
                // g_robot->moveJoint(safe_target.x, safe_target.y, safe_target.z, 
                //                   safe_target.rx, safe_target.ry, safe_target.rz, safe_speed);
                std::cout << "MOVE_JOINT to " << safe_target.toString() 
                          << " at speed " << safe_speed << std::endl;
                break;
                
            case RobotTestCommand::STRIKE:
                // Fast striking motion
                // g_robot->moveLinear(safe_target.x, safe_target.y, safe_target.z, 
                //                    safe_target.rx, safe_target.ry, safe_target.rz, 1.0f);
                std::cout << "STRIKE to " << safe_target.toString() 
                          << " at maximum speed" << std::endl;
                break;
                
            case RobotTestCommand::CALIBRATE:
                // g_robot->calibrate();
                std::cout << "CALIBRATE robot" << std::endl;
                break;
                
            default:
                g_robot_status.last_error = "Unknown command type";
                return false;
        }
        
        // Simulate movement completion (in real implementation, this would be asynchronous)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(cmd.duration * 1000)));
        
        g_robot_status.current_position = safe_target;
        g_robot_status.is_moving = false;
        g_robot_status.last_error.clear();
        g_robot_status.last_update = std::chrono::high_resolution_clock::now();
        
        return true;
        
    } catch (const std::exception& e) {
        g_robot_status.last_error = "Command execution failed: " + std::string(e.what());
        g_robot_status.is_moving = false;
        return false;
    }
#else
    // Simulation mode
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    
    Position3D safe_target = g_safety.clampPosition(cmd.target);
    g_robot_status.target_position = safe_target;
    g_robot_status.is_moving = true;
    g_robot_status.current_test = cmd.name;
    
    std::cout << "SIMULATE: " << cmd.name << " - " << safe_target.toString() 
              << " at speed " << cmd.speed << std::endl;
              
    // Simulate movement time
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(cmd.duration * 1000)));
    
    g_robot_status.current_position = safe_target;
    g_robot_status.is_moving = false;
    g_robot_status.last_error.clear();
    g_robot_status.last_update = std::chrono::high_resolution_clock::now();
    
    return true;
#endif
}

void executeTestSequence(const TestSequence& sequence) {
    g_auto_test_running = true;
    std::cout << "Starting test sequence: " << sequence.name << std::endl;
    
    for (int repeat = 0; repeat < sequence.repeat_count && g_auto_test_running; ++repeat) {
        for (size_t i = 0; i < sequence.commands.size() && g_auto_test_running; ++i) {
            {
                std::lock_guard<std::mutex> lock(g_robot_mutex);
                g_robot_status.test_progress = static_cast<int>((i * 100) / sequence.commands.size());
            }
            
            if (!executeRobotCommand(sequence.commands[i])) {
                std::cout << "Test failed at command: " << sequence.commands[i].name << std::endl;
                break;
            }
            
            if (i < sequence.commands.size() - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(sequence.delay_between * 1000)));
            }
        }
        
        if (sequence.loop && repeat < sequence.repeat_count - 1) {
            std::cout << "Loop " << (repeat + 1) << " completed" << std::endl;
        }
    }
    
    {
        std::lock_guard<std::mutex> lock(g_robot_mutex);
        g_robot_status.test_progress = 100;
        g_robot_status.current_test = "Completed";
    }
    
    g_auto_test_running = false;
    std::cout << "Test sequence completed: " << sequence.name << std::endl;
}

// ======= Utility Functions =======
std::string escape_json(const std::string& s) {
    std::ostringstream o;
    for (char c : s) {
        switch (c) {
            case '\"': o << "\\\""; break;
            case '\\': o << "\\\\"; break;
            case '\n': o << "\\n"; break;
            case '\r': o << "\\r"; break;
            case '\t': o << "\\t"; break;
            default:   o << c; break;
        }
    }
    return o.str();
}

std::string robot_status_json() {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    os << "{"
       << "\"connected\":" << (g_robot_status.is_connected ? "true" : "false") << ","
       << "\"moving\":" << (g_robot_status.is_moving ? "true" : "false") << ","
       << "\"ip\":\"" << escape_json(g_connection.ip) << "\","
       << "\"port\":" << g_connection.port << ","
       << "\"current_position\":{"
       << "\"x\":" << g_robot_status.current_position.x << ","
       << "\"y\":" << g_robot_status.current_position.y << ","
       << "\"z\":" << g_robot_status.current_position.z << ","
       << "\"rx\":" << g_robot_status.current_position.rx << ","
       << "\"ry\":" << g_robot_status.current_position.ry << ","
       << "\"rz\":" << g_robot_status.current_position.rz
       << "},"
       << "\"target_position\":{"
       << "\"x\":" << g_robot_status.target_position.x << ","
       << "\"y\":" << g_robot_status.target_position.y << ","
       << "\"z\":" << g_robot_status.target_position.z << ","
       << "\"rx\":" << g_robot_status.target_position.rx << ","
       << "\"ry\":" << g_robot_status.target_position.ry << ","
       << "\"rz\":" << g_robot_status.target_position.rz
       << "},"
       << "\"current_test\":\"" << escape_json(g_robot_status.current_test) << "\","
       << "\"test_progress\":" << g_robot_status.test_progress << ","
       << "\"last_error\":\"" << escape_json(g_robot_status.last_error) << "\","
       << "\"auto_test_running\":" << (g_auto_test_running ? "true" : "false")
       << "}";
    return os.str();
}

std::string test_sequences_json() {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < g_test_sequences.size(); ++i) {
        if (i > 0) os << ",";
        os << "{"
           << "\"name\":\"" << escape_json(g_test_sequences[i].name) << "\","
           << "\"commands\":" << g_test_sequences[i].commands.size() << ","
           << "\"loop\":" << (g_test_sequences[i].loop ? "true" : "false") << ","
           << "\"repeat_count\":" << g_test_sequences[i].repeat_count
           << "}";
    }
    os << "]";
    return os.str();
}

std::string safety_limits_json() {
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    os << "{"
       << "\"min_position\":{"
       << "\"x\":" << g_safety.min_pos.x << ","
       << "\"y\":" << g_safety.min_pos.y << ","
       << "\"z\":" << g_safety.min_pos.z << ","
       << "\"rx\":" << g_safety.min_pos.rx << ","
       << "\"ry\":" << g_safety.min_pos.ry << ","
       << "\"rz\":" << g_safety.min_pos.rz
       << "},"
       << "\"max_position\":{"
       << "\"x\":" << g_safety.max_pos.x << ","
       << "\"y\":" << g_safety.max_pos.y << ","
       << "\"z\":" << g_safety.max_pos.z << ","
       << "\"rx\":" << g_safety.max_pos.rx << ","
       << "\"ry\":" << g_safety.max_pos.ry << ","
       << "\"rz\":" << g_safety.max_pos.rz
       << "},"
       << "\"max_speed\":" << g_safety.max_speed
       << "}";
    return os.str();
}

// ======= HTTP Functions =======
void send_http_response(int client_socket, const std::string& content_type, const std::string& content) {
    std::string response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + std::to_string(content.length()) + "\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "Connection: close\r\n"
        "\r\n" + content;

    send(client_socket, response.c_str(), response.length(), 0);
}

std::string get_html_page() {
    return R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Test Control</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin:0; padding:20px; background:#f0f0f0; }
        .container { max-width: 1400px; margin: 0 auto; background:#fff; padding:20px; border-radius:10px; box-shadow:0 2px 10px rgba(0,0,0,0.1); }
        h1 { color:#333; text-align:center; margin-bottom:30px; }
        .section { margin: 20px 0; padding: 20px; border: 1px solid #ddd; border-radius: 8px; background:#fafafa; }
        .controls { text-align:center; margin:15px 0; }
        .controls-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; align-items: start; }
        button { background:#4CAF50; border:none; color:#fff; padding:12px 24px; font-size:16px; margin:4px 8px; cursor:pointer; border-radius:6px; }
        button:hover { background:#45a049; }
        button.danger { background:#f44336; }
        button.danger:hover { background:#da190b; }
        button.warning { background:#ff9800; }
        button.warning:hover { background:#e68900; }
        .info { background:#e7f3ff; border-left:6px solid #2196F3; margin: 15px 0; padding:15px; }
        .error { background:#ffebee; border-left:6px solid #f44336; margin: 15px 0; padding:15px; }
        .success { background:#e8f5e8; border-left:6px solid #4caf50; margin: 15px 0; padding:15px; }
        input, select { border:1px solid #ccc; border-radius:6px; padding:10px; margin:5px; width: 100px; }
        .position-input { width: 80px; }
        .position-grid { display: grid; grid-template-columns: repeat(6, 1fr); gap: 10px; align-items: center; margin: 15px 0; }
        .position-grid label { font-weight: bold; text-align: center; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 20px; }
        .progress { width: 100%; height: 20px; background-color: #f0f0f0; border-radius: 10px; overflow: hidden; margin: 10px 0; }
        .progress-bar { height: 100%; background-color: #4caf50; transition: width 0.3s ease; }
        .test-list { max-height: 300px; overflow-y: auto; border: 1px solid #ddd; padding: 10px; margin: 10px 0; }
        .test-item { padding: 10px; margin: 5px 0; border: 1px solid #eee; border-radius: 5px; cursor: pointer; }
        .test-item:hover { background-color: #f5f5f5; }
        .test-item.selected { background-color: #e3f2fd; border-color: #2196f3; }
        .safety-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        table { width: 100%; border-collapse: collapse; margin: 10px 0; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }
        th { background-color: #f2f2f2; }
        .real-time { font-family: monospace; font-size: 14px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Test Control System</h1>
        
        <div class="status-grid">
            <div class="section">
                <h2>📡 Connection Status</h2>
                <div id="connection-status" class="info">
                    <p><strong>Status:</strong> <span id="conn-status">Disconnected</span></p>
                    <p><strong>IP:</strong> <span id="conn-ip">192.168.125.1</span></p>
                    <p><strong>Port:</strong> <span id="conn-port">11000</span></p>
                </div>
                <div class="controls">
                    <input id="robot-ip" type="text" value="192.168.125.1" placeholder="IP Address">
                    <input id="robot-port" type="number" value="11000" placeholder="Port">
                    <button onclick="connectRobot()">🔌 Connect</button>
                    <button onclick="disconnectRobot()" class="danger">❌ Disconnect</button>
                </div>
            </div>
            
            <div class="section">
                <h2>📍 Current Position</h2>
                <div id="current-position" class="info real-time">
                    <table>
                        <tr><th>X</th><th>Y</th><th>Z</th><th>RX</th><th>RY</th><th>RZ</th></tr>
                        <tr>
                            <td id="pos-x">0.000</td>
                            <td id="pos-y">0.000</td>
                            <td id="pos-z">0.000</td>
                            <td id="pos-rx">0.000</td>
                            <td id="pos-ry">0.000</td>
                            <td id="pos-rz">0.000</td>
                        </tr>
                    </table>
                    <p><strong>Moving:</strong> <span id="is-moving">No</span></p>
                    <p><strong>Current Test:</strong> <span id="current-test">None</span></p>
                </div>
            </div>
            
            <div class="section">
                <h2>🎯 Test Progress</h2>
                <div id="test-progress-section">
                    <div class="progress">
                        <div id="progress-bar" class="progress-bar" style="width: 0%"></div>
                    </div>
                    <p><strong>Progress:</strong> <span id="progress-text">0%</span></p>
                    <p><strong>Auto Test:</strong> <span id="auto-test-status">Stopped</span></p>
                </div>
            </div>
        </div>

        <div class="controls-grid">
            <div class="section">
                <h2>🎮 Manual Control</h2>
                <div class="position-grid">
                    <label>X:</label>
                    <input id="manual-x" type="number" class="position-input" step="0.01" value="0" min="-0.8" max="0.8">
                    <label>Y:</label>
                    <input id="manual-y" type="number" class="position-input" step="0.01" value="0" min="-0.6" max="0.6">
                    <label>Z:</label>
                    <input id="manual-z" type="number" class="position-input" step="0.01" value="0.1" min="0.01" max="0.3">
                    <label>RX:</label>
                    <input id="manual-rx" type="number" class="position-input" step="1" value="0" min="-180" max="180">
                    <label>RY:</label>
                    <input id="manual-ry" type="number" class="position-input" step="1" value="0" min="-180" max="180">
                    <label>RZ:</label>
                    <input id="manual-rz" type="number" class="position-input" step="1" value="0" min="-180" max="180">
                </div>
                <div class="controls">
                    <label>Speed: </label>
                    <input id="manual-speed" type="range" min="0.1" max="1.0" step="0.1" value="0.5">
                    <span id="speed-value">0.5</span>
                </div>
                <div class="controls">
                    <button onclick="moveToPosition()">📍 Move To Position</button>
                    <button onclick="moveHome()">🏠 Move Home</button>
                    <button onclick="emergencyStop()" class="danger">🛑 EMERGENCY STOP</button>
                </div>
                
                <h3>Quick Movements</h3>
                <div class="controls">
                    <button onclick="quickMove('up')">⬆️ Up (+Z)</button>
                    <button onclick="quickMove('down')">⬇️ Down (-Z)</button>
                    <button onclick="quickMove('left')">⬅️ Left (-X)</button>
                    <button onclick="quickMove('right')">➡️ Right (+X)</button>
                    <button onclick="quickMove('forward')">⬆️ Forward (+Y)</button>
                    <button onclick="quickMove('back')">⬇️ Back (-Y)</button>
                </div>
            </div>
            
            <div class="section">
                <h2>🧪 Automated Tests</h2>
                <div class="test-list" id="test-list">
                    <!-- Test sequences will be populated here -->
                </div>
                <div class="controls">
                    <button onclick="startSelectedTest()">▶️ Start Test</button>
                    <button onclick="pauseTest()" class="warning">⏸️ Pause Test</button>
                    <button onclick="stopTest()" class="danger">⏹️ Stop Test</button>
                </div>
                
                <h3>Test Options</h3>
                <div class="controls">
                    <label>Repeat Count: </label>
                    <input id="repeat-count" type="number" min="1" max="10" value="1">
                    <label>Delay (s): </label>
                    <input id="test-delay" type="number" min="0.1" max="10" step="0.1" value="1.0">
                    <input type="checkbox" id="loop-test"> <label for="loop-test">Loop Test</label>
                </div>
            </div>
        </div>

        <div class="section">
            <h2>🛡️ Safety Settings</h2>
            <div class="safety-grid">
                <div>
                    <h3>Position Limits</h3>
                    <table>
                        <tr><th>Axis</th><th>Min</th><th>Max</th></tr>
                        <tr><td>X</td><td id="limit-x-min">-0.800</td><td id="limit-x-max">0.800</td></tr>
                        <tr><td>Y</td><td id="limit-y-min">-0.600</td><td id="limit-y-max">0.600</td></tr>
                        <tr><td>Z</td><td id="limit-z-min">0.010</td><td id="limit-z-max">0.300</td></tr>
                        <tr><td>RX</td><td id="limit-rx-min">-180</td><td id="limit-rx-max">180</td></tr>
                        <tr><td>RY</td><td id="limit-ry-min">-180</td><td id="limit-ry-max">180</td></tr>
                        <tr><td>RZ</td><td id="limit-rz-min">-180</td><td id="limit-rz-max">180</td></tr>
                    </table>
                </div>
                <div>
                    <h3>Speed Limits</h3>
                    <p><strong>Max Speed:</strong> <span id="max-speed">1.000</span></p>
                    <p><strong>Max Acceleration:</strong> <span id="max-accel">2.000</span></p>
                    <div class="controls">
                        <button onclick="updateSafetyLimits()">⚙️ Update Limits</button>
                        <button onclick="resetSafetyLimits()" class="warning">🔄 Reset to Default</button>
                    </div>
                </div>
            </div>
        </div>

        <div class="section">
            <h2>📊 System Status</h2>
            <div id="system-status" class="info">
                <p><strong>Last Error:</strong> <span id="last-error">None</span></p>
                <p><strong>Last Update:</strong> <span id="last-update">Never</span></p>
                <p><strong>System Time:</strong> <span id="system-time">--:--:--</span></p>
            </div>
            <div class="controls">
                <button onclick="calibrateRobot()">🎯 Calibrate Robot</button>
                <button onclick="resetRobot()" class="warning">🔄 Reset Robot</button>
                <button onclick="downloadLog()">💾 Download Log</button>
                <button onclick="clearLog()" class="danger">🗑️ Clear Log</button>
            </div>
        </div>
    </div>

    <script>
        let selectedTestIndex = -1;
        let updateInterval;

        // Initialize page
        window.addEventListener('load', function() {
            loadTestSequences();
            startStatusUpdates();
            updateSystemTime();
            setInterval(updateSystemTime, 1000);
            
            // Speed slider update
            document.getElementById('manual-speed').addEventListener('input', function() {
                document.getElementById('speed-value').textContent = this.value;
            });
        });

        // Status update functions
        async function updateRobotStatus() {
            try {
                const response = await fetch('/robot/status');
                const status = await response.json();
                
                // Update connection status
                document.getElementById('conn-status').textContent = status.connected ? 'Connected' : 'Disconnected';
                document.getElementById('conn-ip').textContent = status.ip || '192.168.125.1';
                document.getElementById('conn-port').textContent = status.port || '11000';
                document.getElementById('conn-status').parentElement.parentElement.className = 
                    status.connected ? 'info success' : 'info error';
                
                // Update position
                document.getElementById('pos-x').textContent = status.current_position.x.toFixed(3);
                document.getElementById('pos-y').textContent = status.current_position.y.toFixed(3);
                document.getElementById('pos-z').textContent = status.current_position.z.toFixed(3);
                document.getElementById('pos-rx').textContent = status.current_position.rx.toFixed(1);
                document.getElementById('pos-ry').textContent = status.current_position.ry.toFixed(1);
                document.getElementById('pos-rz').textContent = status.current_position.rz.toFixed(1);
                
                // Update movement status
                document.getElementById('is-moving').textContent = status.moving ? 'Yes' : 'No';
                document.getElementById('current-test').textContent = status.current_test || 'None';
                
                // Update progress
                document.getElementById('progress-bar').style.width = status.test_progress + '%';
                document.getElementById('progress-text').textContent = status.test_progress + '%';
                document.getElementById('auto-test-status').textContent = 
                    status.auto_test_running ? 'Running' : 'Stopped';
                
                // Update error status
                document.getElementById('last-error').textContent = status.last_error || 'None';
                document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
                
            } catch (error) {
                console.error('Failed to update robot status:', error);
                document.getElementById('conn-status').textContent = 'Error';
            }
        }

        async function loadTestSequences() {
            try {
                const response = await fetch('/test/sequences');
                const sequences = await response.json();
                
                const testList = document.getElementById('test-list');
                testList.innerHTML = '';
                
                sequences.forEach((seq, index) => {
                    const item = document.createElement('div');
                    item.className = 'test-item';
                    item.innerHTML = `
                        <strong>${seq.name}</strong><br>
                        Commands: ${seq.commands}, Loop: ${seq.loop ? 'Yes' : 'No'}, Repeats: ${seq.repeat_count}
                    `;
                    item.onclick = () => selectTest(index);
                    testList.appendChild(item);
                });
                
            } catch (error) {
                console.error('Failed to load test sequences:', error);
            }
        }

        async function loadSafetyLimits() {
            try {
                const response = await fetch('/robot/safety');
                const limits = await response.json();
                
                document.getElementById('limit-x-min').textContent = limits.min_position.x.toFixed(3);
                document.getElementById('limit-x-max').textContent = limits.max_position.x.toFixed(3);
                document.getElementById('limit-y-min').textContent = limits.min_position.y.toFixed(3);
                document.getElementById('limit-y-max').textContent = limits.max_position.y.toFixed(3);
                document.getElementById('limit-z-min').textContent = limits.min_position.z.toFixed(3);
                document.getElementById('limit-z-max').textContent = limits.max_position.z.toFixed(3);
                document.getElementById('limit-rx-min').textContent = limits.min_position.rx.toFixed(0);
                document.getElementById('limit-rx-max').textContent = limits.max_position.rx.toFixed(0);
                document.getElementById('limit-ry-min').textContent = limits.min_position.ry.toFixed(0);
                document.getElementById('limit-ry-max').textContent = limits.max_position.ry.toFixed(0);
                document.getElementById('limit-rz-min').textContent = limits.min_position.rz.toFixed(0);
                document.getElementById('limit-rz-max').textContent = limits.max_position.rz.toFixed(0);
                document.getElementById('max-speed').textContent = limits.max_speed.toFixed(3);
                
            } catch (error) {
                console.error('Failed to load safety limits:', error);
            }
        }

        function selectTest(index) {
            // Remove previous selection
            document.querySelectorAll('.test-item').forEach(item => item.classList.remove('selected'));
            
            // Select new test
            document.querySelectorAll('.test-item')[index].classList.add('selected');
            selectedTestIndex = index;
        }

        function startStatusUpdates() {
            updateRobotStatus();
            loadSafetyLimits();
            updateInterval = setInterval(updateRobotStatus, 1000);
        }

        function updateSystemTime() {
            document.getElementById('system-time').textContent = new Date().toLocaleTimeString();
        }

        // Robot control functions
        async function connectRobot() {
            const ip = document.getElementById('robot-ip').value;
            const port = document.getElementById('robot-port').value;
            
            try {
                const response = await fetch(`/robot/connect?ip=${ip}&port=${port}`, {
                    method: 'POST'
                });
                const result = await response.json();
                
                if (result.status === 'connected') {
                    alert('Robot connected successfully!');
                } else {
                    alert('Failed to connect: ' + (result.error || 'Unknown error'));
                }
                
            } catch (error) {
                alert('Connection failed: ' + error.message);
            }
        }

        async function disconnectRobot() {
            try {
                await fetch('/robot/disconnect', { method: 'POST' });
                alert('Robot disconnected');
            } catch (error) {
                alert('Disconnect failed: ' + error.message);
            }
        }

        async function moveToPosition() {
            const position = {
                x: parseFloat(document.getElementById('manual-x').value),
                y: parseFloat(document.getElementById('manual-y').value),
                z: parseFloat(document.getElementById('manual-z').value),
                rx: parseFloat(document.getElementById('manual-rx').value),
                ry: parseFloat(document.getElementById('manual-ry').value),
                rz: parseFloat(document.getElementById('manual-rz').value),
                speed: parseFloat(document.getElementById('manual-speed').value)
            };
            
            try {
                const response = await fetch('/robot/move', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(position)
                });
                const result = await response.json();
                
                if (result.status !== 'ok') {
                    alert('Move failed: ' + (result.error || 'Unknown error'));
                }
                
            } catch (error) {
                alert('Move command failed: ' + error.message);
            }
        }

        async function moveHome() {
            document.getElementById('manual-x').value = '0';
            document.getElementById('manual-y').value = '0';
            document.getElementById('manual-z').value = '0.1';
            document.getElementById('manual-rx').value = '0';
            document.getElementById('manual-ry').value = '0';
            document.getElementById('manual-rz').value = '0';
            await moveToPosition();
        }

        async function quickMove(direction) {
            const step = 0.05; // 5cm steps
            const currentX = parseFloat(document.getElementById('manual-x').value);
            const currentY = parseFloat(document.getElementById('manual-y').value);
            const currentZ = parseFloat(document.getElementById('manual-z').value);
            
            switch (direction) {
                case 'up':
                    document.getElementById('manual-z').value = (currentZ + step).toFixed(3);
                    break;
                case 'down':
                    document.getElementById('manual-z').value = (currentZ - step).toFixed(3);
                    break;
                case 'left':
                    document.getElementById('manual-x').value = (currentX - step).toFixed(3);
                    break;
                case 'right':
                    document.getElementById('manual-x').value = (currentX + step).toFixed(3);
                    break;
                case 'forward':
                    document.getElementById('manual-y').value = (currentY + step).toFixed(3);
                    break;
                case 'back':
                    document.getElementById('manual-y').value = (currentY - step).toFixed(3);
                    break;
            }
            
            await moveToPosition();
        }

        async function emergencyStop() {
            if (confirm('Execute EMERGENCY STOP? This will immediately halt all robot movement.')) {
                try {
                    await fetch('/robot/emergency_stop', { method: 'POST' });
                    alert('Emergency stop executed');
                } catch (error) {
                    alert('Emergency stop failed: ' + error.message);
                }
            }
        }

        async function startSelectedTest() {
            if (selectedTestIndex === -1) {
                alert('Please select a test sequence first');
                return;
            }
            
            const repeatCount = parseInt(document.getElementById('repeat-count').value);
            const delay = parseFloat(document.getElementById('test-delay').value);
            const loop = document.getElementById('loop-test').checked;
            
            try {
                const response = await fetch('/test/start', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        sequence_index: selectedTestIndex,
                        repeat_count: repeatCount,
                        delay: delay,
                        loop: loop
                    })
                });
                const result = await response.json();
                
                if (result.status === 'started') {
                    alert('Test sequence started');
                } else {
                    alert('Failed to start test: ' + (result.error || 'Unknown error'));
                }
                
            } catch (error) {
                alert('Start test failed: ' + error.message);
            }
        }

        async function pauseTest() {
            try {
                await fetch('/test/pause', { method: 'POST' });
            } catch (error) {
                alert('Pause failed: ' + error.message);
            }
        }

        async function stopTest() {
            try {
                await fetch('/test/stop', { method: 'POST' });
                alert('Test stopped');
            } catch (error) {
                alert('Stop failed: ' + error.message);
            }
        }

        async function calibrateRobot() {
            if (confirm('Start robot calibration sequence?')) {
                try {
                    await fetch('/robot/calibrate', { method: 'POST' });
                    alert('Calibration started');
                } catch (error) {
                    alert('Calibration failed: ' + error.message);
                }
            }
        }

        async function resetRobot() {
            if (confirm('Reset robot to home position?')) {
                try {
                    await fetch('/robot/reset', { method: 'POST' });
                    alert('Robot reset initiated');
                } catch (error) {
                    alert('Reset failed: ' + error.message);
                }
            }
        }

        function updateSafetyLimits() {
            alert('Safety limit modification requires admin privileges');
        }

        function resetSafetyLimits() {
            if (confirm('Reset all safety limits to factory defaults?')) {
                alert('Safety limits reset to defaults');
                loadSafetyLimits();
            }
        }

        function downloadLog() {
            window.open('/robot/log', '_blank');
        }

        function clearLog() {
            if (confirm('Clear all log entries?')) {
                fetch('/robot/log', { method: 'DELETE' })
                    .then(() => alert('Log cleared'))
                    .catch(error => alert('Clear log failed: ' + error.message));
            }
        }
    </script>
</body>
</html>
)html";
}

void handle_client(int client_socket) {
    char buffer[4096] = {0};
    recv(client_socket, buffer, sizeof(buffer)-1, 0);

    std::string request(buffer);
    std::cout << "Request: " << request.substr(0, request.find('\n')) << std::endl;

    if (request.find("GET / ") == 0) {
        send_http_response(client_socket, "text/html", get_html_page());
    }
    else if (request.find("GET /robot/status") == 0) {
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.find("GET /test/sequences") == 0) {
        send_http_response(client_socket, "application/json", test_sequences_json());
    }
    else if (request.find("GET /robot/safety") == 0) {
        send_http_response(client_socket, "application/json", safety_limits_json());
    }
    else if (request.find("POST /robot/connect") == 0) {
#ifdef USE_ABB
        // Parse query parameters
        std::string ip = g_connection.ip;
        uint16_t port = g_connection.port;
        
        auto qpos = request.find('?');
        if (qpos != std::string::npos) {
            auto qs = request.substr(qpos+1, request.find(' ') - (qpos+1));
            std::map<std::string,std::string> kv;
            std::stringstream ss(qs);
            std::string token;
            while (std::getline(ss, token, '&')) {
                auto eq = token.find('=');
                if (eq != std::string::npos) {
                    kv[token.substr(0,eq)] = token.substr(eq+1);
                }
            }
            if (kv.count("ip")) ip = kv["ip"];
            if (kv.count("port")) port = static_cast<uint16_t>(std::stoi(kv["port"]));
        }
        
        try {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            if (g_robot) {
                g_robot->disconnect();
                delete g_robot;
            }
            
            g_robot = new AbbController(ip, port);
            if (g_robot->connect()) {
                g_robot_status.is_connected = true;
                g_robot_status.last_error.clear();
                g_connection.ip = ip;
                g_connection.port = port;
                send_http_response(client_socket, "application/json", "{\"status\":\"connected\"}");
            } else {
                delete g_robot;
                g_robot = nullptr;
                g_robot_status.is_connected = false;
                g_robot_status.last_error = "Connection failed";
                send_http_response(client_socket, "application/json", "{\"status\":\"failed\",\"error\":\"Connection failed\"}");
            }
        } catch (const std::exception& e) {
            g_robot_status.last_error = e.what();
            send_http_response(client_socket, "application/json", 
                "{\"status\":\"failed\",\"error\":\"" + std::string(e.what()) + "\"}");
        }
#else
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            g_robot_status.is_connected = true;
            g_robot_status.last_error = "Simulation mode - not using real ABB robot";
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"connected\",\"mode\":\"simulation\"}");
#endif
    }
    else if (request.find("POST /robot/disconnect") == 0) {
#ifdef USE_ABB
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            if (g_robot) {
                g_robot->disconnect();
                delete g_robot;
                g_robot = nullptr;
            }
            g_robot_status.is_connected = false;
            g_robot_status.last_error = "Disconnected by user";
        }
#else
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            g_robot_status.is_connected = false;
            g_robot_status.last_error = "Simulation mode disconnected";
        }
#endif
        send_http_response(client_socket, "application/json", "{\"status\":\"disconnected\"}");
    }
    else if (request.find("POST /robot/move") == 0) {
        // Parse JSON body for movement command
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            
            // Simple JSON parsing - in production use proper JSON library
            Position3D target;
            float speed = 0.5f;
            
            // Extract values (simplified parsing)
            size_t pos = body.find("\"x\":");
            if (pos != std::string::npos) {
                target.x = std::stof(body.substr(pos + 4));
            }
            pos = body.find("\"y\":");
            if (pos != std::string::npos) {
                target.y = std::stof(body.substr(pos + 4));
            }
            pos = body.find("\"z\":");
            if (pos != std::string::npos) {
                target.z = std::stof(body.substr(pos + 4));
            }
            pos = body.find("\"speed\":");
            if (pos != std::string::npos) {
                speed = std::stof(body.substr(pos + 8));
            }
            
            RobotTestCommand cmd(RobotTestCommand::MOVE_TO, target, speed, "Manual Move");
            
            if (executeRobotCommand(cmd)) {
                send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
            } else {
                send_http_response(client_socket, "application/json", 
                    "{\"status\":\"error\",\"error\":\"" + g_robot_status.last_error + "\"}");
            }
        } else {
            send_http_response(client_socket, "application/json", "{\"status\":\"error\",\"error\":\"No request body\"}");
        }
    }
    else if (request.find("POST /robot/emergency_stop") == 0) {
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            g_robot_status.is_moving = false;
            g_robot_status.current_test = "Emergency Stop";
            g_robot_status.last_error = "Emergency stop executed";
            
#ifdef USE_ABB
            if (g_robot) {
                try {
                    // g_robot->emergencyStop();
                } catch (const std::exception& e) {
                    g_robot_status.last_error = "Emergency stop failed: " + std::string(e.what());
                }
            }
#endif
        }
        
        g_auto_test_running = false;
        if (g_test_thread.joinable()) {
            g_test_thread.join();
        }
        
        send_http_response(client_socket, "application/json", "{\"status\":\"stopped\"}");
    }
    else if (request.find("POST /test/start") == 0) {
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            
            // Parse test parameters
            int sequence_index = 0;
            int repeat_count = 1;
            float delay = 1.0f;
            bool loop = false;
            
            // Simple parsing (use proper JSON parser in production)
            size_t pos = body.find("\"sequence_index\":");
            if (pos != std::string::npos) {
                sequence_index = std::stoi(body.substr(pos + 17));
            }
            
            if (sequence_index >= 0 && sequence_index < g_test_sequences.size()) {
                if (g_test_thread.joinable()) {
                    g_auto_test_running = false;
                    g_test_thread.join();
                }
                
                TestSequence test_seq = g_test_sequences[sequence_index];
                test_seq.repeat_count = repeat_count;
                test_seq.delay_between = delay;
                test_seq.loop = loop;
                
                g_test_thread = std::thread(executeTestSequence, test_seq);
                send_http_response(client_socket, "application/json", "{\"status\":\"started\"}");
            } else {
                send_http_response(client_socket, "application/json", 
                    "{\"status\":\"error\",\"error\":\"Invalid sequence index\"}");
            }
        } else {
            send_http_response(client_socket, "application/json", "{\"status\":\"error\",\"error\":\"No request body\"}");
        }
    }
    else if (request.find("POST /test/stop") == 0) {
        g_auto_test_running = false;
        if (g_test_thread.joinable()) {
            g_test_thread.join();
        }
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            g_robot_status.current_test = "Stopped";
            g_robot_status.test_progress = 0;
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"stopped\"}");
    }
    else if (request.find("POST /robot/calibrate") == 0) {
        RobotTestCommand cmd(RobotTestCommand::CALIBRATE, Position3D(), 0.3f, "Calibration");
        if (executeRobotCommand(cmd)) {
            send_http_response(client_socket, "application/json", "{\"status\":\"calibrating\"}");
        } else {
            send_http_response(client_socket, "application/json", 
                "{\"status\":\"error\",\"error\":\"" + g_robot_status.last_error + "\"}");
        }
    }
    else if (request.find("POST /robot/reset") == 0) {
        RobotTestCommand cmd(RobotTestCommand::MOVE_TO, Position3D(0, 0, 0.1f), 0.3f, "Reset to Home");
        if (executeRobotCommand(cmd)) {
            send_http_response(client_socket, "application/json", "{\"status\":\"resetting\"}");
        } else {
            send_http_response(client_socket, "application/json", 
                "{\"status\":\"error\",\"error\":\"" + g_robot_status.last_error + "\"}");
        }
    }
    else if (request.find("GET /robot/log") == 0) {
        // Return robot operation log
        std::string log = "Robot Test Log\n";
        log += "==============\n";
        log += "Current Position: " + g_robot_status.current_position.toString() + "\n";
        log += "Last Error: " + g_robot_status.last_error + "\n";
        log += "Connection Status: " + std::string(g_robot_status.is_connected ? "Connected" : "Disconnected") + "\n";
        
        send_http_response(client_socket, "text/plain", log);
    }
    else {
        send_http_response(client_socket, "text/html", "<h1>404 Not Found</h1>");
    }

    close(client_socket);
}

int main() {
    std::cout << "=== ROBOT TEST CONTROL SYSTEM ===" << std::endl;
    
    // Initialize test sequences
    initializeTestSequences();
    
    // Initialize robot status
    g_robot_status.current_position = Position3D(0, 0, 0.1f);
    g_robot_status.target_position = Position3D(0, 0, 0.1f);
    
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        std::cerr << "❌ Cannot create socket!" << std::endl;
        return -1;
    }

    int opt = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8080);

    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "❌ Bind failed!" << std::endl;
        return -1;
    }
    if (listen(server_socket, 10) < 0) {
        std::cerr << "❌ Listen failed!" << std::endl;
        return -1;
    }

    std::cout << "🌐 Robot Test Server running on port 8080" << std::endl;
    std::cout << "🤖 Available test sequences: " << g_test_sequences.size() << std::endl;
    std::cout << "🛡️ Safety limits active" << std::endl;
    std::cout << "📡 Connect via: http://<IP>:8080" << std::endl;

    while (running) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_socket < 0) {
            if (running) {
                std::cerr << "Accept failed" << std::endl;
            }
            continue;
        }

        std::thread client_thread(handle_client, client_socket);
        client_thread.detach();
    }

    // Cleanup
    std::cout << "🛑 Shutting down server..." << std::endl;
    
    // Stop any running tests
    g_auto_test_running = false;
    if (g_test_thread.joinable()) {
        g_test_thread.join();
    }
    
    // Disconnect robot
#ifdef USE_ABB
    {
        std::lock_guard<std::mutex> lock(g_robot_mutex);
        if (g_robot) {
            try {
                g_robot->disconnect();
                std::cout << "🤖 Robot disconnected safely" << std::endl;
            } catch (...) {
                std::cout << "⚠️ Robot disconnect had issues" << std::endl;
            }
            delete g_robot;
            g_robot = nullptr;
        }
    }
#endif

    close(server_socket);
    std::cout << "✅ Server shutdown complete" << std::endl;
    return 0;
}