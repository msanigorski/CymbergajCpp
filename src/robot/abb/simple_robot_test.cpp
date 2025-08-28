#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <iomanip>
#include <vector>
#include <atomic>
#include <mutex>
#include <cmath>

#ifdef USE_ABB
#include "abb_comm.h"
#endif

// Robot connection settings
const std::string ROBOT_IP = "10.25.74.172";
const uint16_t MOTION_PORT = 5000;    // SERVER.mod
const uint16_t LOGGER_PORT = 5001;    // LOGGER.mod

// Test positions (adjust according to your robot workspace)
struct Position {
    double x, y, z, q0, qx, qy, qz;  // Cartesian + Quaternion
    std::string name;
    
    // Default constructor
    Position() : x(0), y(0), z(0), q0(1), qx(0), qy(0), qz(0), name("Default") {}
    
    // Direct quaternion constructor
    Position(double x, double y, double z, double q0, double qx, double qy, double qz, const std::string& name)
        : x(x), y(y), z(z), q0(q0), qx(qx), qy(qy), qz(qz), name(name) {}
    
    // Convenience constructor with identity orientation (no rotation)
    Position(double x, double y, double z, const std::string& name = "")
        : x(x), y(y), z(z), q0(1.0), qx(0.0), qy(0.0), qz(0.0), name(name) {}
        
    void print() const {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << name << ": [" << x << ", " << y << ", " << z << "] Q[" << q0 << ", " << qx << ", " << qy << ", " << qz << "]";
    }
};

class RobotTestWithLogger {
private:
#ifdef USE_ABB
    AbbRobotController* robot;
#endif
    bool connected;
    std::atomic<bool> position_monitoring;
    RobotPosition last_position;
    std::mutex last_position_mutex;
    
public:
    RobotTestWithLogger() : connected(false), position_monitoring(false) {
#ifdef USE_ABB
        robot = nullptr;
#endif
    }
    
    ~RobotTestWithLogger() {
        disconnect();
    }
    
    // Position update callback
    void onPositionUpdate(const RobotPosition& pos) {
        std::lock_guard<std::mutex> lock(last_position_mutex);
        last_position = pos;
        
        if (position_monitoring.load()) {
            std::cout << "📍 Live Position: X=" << std::fixed << std::setprecision(2) 
                      << pos.x << " Y=" << pos.y << " Z=" << pos.z 
                      << " | J1=" << pos.joint1 << " J2=" << pos.joint2 
                      << " J3=" << pos.joint3 << "        \r" << std::flush;
        }
    }
    
    bool connect() {
        std::cout << "🔌 Attempting to connect to robot at " << ROBOT_IP 
                  << " (Motion:" << MOTION_PORT << ", Logger:" << LOGGER_PORT << ")" << std::endl;
        
#ifdef USE_ABB
        try {
            robot = new AbbRobotController(ROBOT_IP, MOTION_PORT, LOGGER_PORT);
            
            // Set position callback
            robot->setPositionCallback([this](const RobotPosition& pos) {
                this->onPositionUpdate(pos);
            });
            
            if (robot->connect()) {
                connected = true;
                std::cout << "✅ Successfully connected to ABB robot!" << std::endl;
                
                // Test ping
                if (robot->ping()) {
                    std::cout << "🏓 Ping test successful - robot is responding" << std::endl;
                } else {
                    std::cout << "⚠️  Ping test failed - but connection established" << std::endl;
                }
                
                return true;
            } else {
                std::cout << "❌ Failed to connect: robot->connect() returned false" << std::endl;
                delete robot;
                robot = nullptr;
                return false;
            }
            
        } catch (const std::exception& e) {
            std::cout << "❌ Connection failed with exception: " << e.what() << std::endl;
            if (robot) {
                delete robot;
                robot = nullptr;
            }
            return false;
        }
#else
        std::cout << "⚠️  ABB support not compiled (USE_ABB=OFF)" << std::endl;
        std::cout << "🎭 Running in SIMULATION mode" << std::endl;
        connected = true;
        return true;
#endif
    }
    
    void disconnect() {
        if (connected) {
            std::cout << "\n🔌 Disconnecting from robot..." << std::endl;
            stopPositionMonitoring();
            
#ifdef USE_ABB
            if (robot) {
                try {
                    robot->disconnect();
                    delete robot;
                    robot = nullptr;
                } catch (const std::exception& e) {
                    std::cout << "⚠️  Disconnect error: " << e.what() << std::endl;
                }
            }
#endif
            connected = false;
            std::cout << "✅ Disconnected successfully" << std::endl;
        }
    }
    
    bool isConnected() const {
        return connected;
    }
    
    bool moveToPosition(const Position& pos, double tcp_speed = 100.0, double ori_speed = 50.0) {
        if (!connected) {
            std::cout << "❌ Robot not connected!" << std::endl;
            return false;
        }
        
        std::cout << "🎯 Moving to position: ";
        pos.print();
        std::cout << " at speed TCP=" << tcp_speed << "mm/s, ORI=" << ori_speed << "deg/s" << std::endl;
        
#ifdef USE_ABB
        try {
            // Set speed first
            if (!robot->setSpeed(tcp_speed, ori_speed)) {
                std::cout << "⚠️  Failed to set speed, continuing anyway..." << std::endl;
            }
            
            // Move to position
            bool success = robot->moveCartesian(pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz);
            
            if (success) {
                std::cout << "📡 Move command sent successfully" << std::endl;
                std::cout << "⏳ Waiting for movement to complete..." << std::endl;
                
                // Monitor position during movement
                auto start_time = std::chrono::steady_clock::now();
                double initial_distance = -1;
                
                while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(15)) {
                    RobotPosition current = robot->getCurrentPosition();
                    if (current.valid) {
                        double distance = std::sqrt(std::pow(current.x - pos.x, 2) + 
                                             std::pow(current.y - pos.y, 2) + 
                                             std::pow(current.z - pos.z, 2));
                        
                        if (initial_distance < 0) {
                            initial_distance = distance;
                        }
                        
                        std::cout << "📍 Distance to target: " << std::fixed << std::setprecision(3) 
                                  << distance << "m        \r" << std::flush;
                        
                        if (distance < 0.005) {  // 5mm tolerance
                            std::cout << "\n✅ Target reached (within 5mm tolerance)" << std::endl;
                            return true;
                        }
                    }
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                
                std::cout << "\n⏰ Movement timeout (15s) - position may still be updating" << std::endl;
                return true;
                
            } else {
                std::cout << "❌ Failed to send move command" << std::endl;
                return false;
            }
            
        } catch (const std::exception& e) {
            std::cout << "❌ Movement failed with exception: " << e.what() << std::endl;
            return false;
        }
#else
        // Simulation mode
        std::cout << "🎭 SIMULATION: Moving robot to position..." << std::endl;
        for (int i = 0; i < 10; i++) {
            std::cout << "🎭 Movement progress: " << (i+1)*10 << "%\r" << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::cout << "\n✅ SIMULATION: Movement completed" << std::endl;
        return true;
#endif
    }
    
    bool getCurrentPosition(Position& pos) {
        if (!connected) {
            std::cout << "❌ Robot not connected!" << std::endl;
            return false;
        }
        
#ifdef USE_ABB
        try {
            // Wait a bit for logger to get data
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Try to get position from logger first (real-time)
            RobotPosition current = robot->getCurrentPosition();
            if (current.valid) {
                pos.x = current.x;
                pos.y = current.y;
                pos.z = current.z;
                pos.q0 = current.q0;
                pos.qx = current.qx;
                pos.qy = current.qy;
                pos.qz = current.qz;
                pos.name = "Logger";
                std::cout << "✅ Got position from LOGGER" << std::endl;
                return true;
            }
            
            std::cout << "⚠️  No valid data from LOGGER yet, trying SERVER query..." << std::endl;
            
            // Fallback to querying motion server
            double x, y, z, q0, qx, qy, qz;
            if (robot->queryCartesianPosition(x, y, z, q0, qx, qy, qz)) {
                pos.x = x; pos.y = y; pos.z = z;
                pos.q0 = q0; pos.qx = qx; pos.qy = qy; pos.qz = qz;
                pos.name = "Query";
                std::cout << "✅ Got position from SERVER query" << std::endl;
                return true;
            }
            
            std::cout << "❌ Both LOGGER and SERVER query failed" << std::endl;
            return false;
            
        } catch (const std::exception& e) {
            std::cout << "❌ Failed to get current position: " << e.what() << std::endl;
            return false;
        }
#else
        // Simulation mode
        pos = Position(0.3, 0.0, 0.2, "Simulated");
        return true;
#endif
    }
    
    bool getLoggerPositionOnly(Position& pos) {
        if (!connected) {
            std::cout << "❌ Robot not connected!" << std::endl;
            return false;
        }
        
#ifdef USE_ABB
        try {
            // Only try logger - no SERVER fallback
            RobotPosition current = robot->getCurrentPosition();
            if (current.valid) {
                pos.x = current.x;
                pos.y = current.y;
                pos.z = current.z;
                pos.q0 = current.q0;
                pos.qx = current.qx;
                pos.qy = current.qy;
                pos.qz = current.qz;
                pos.name = "Logger";
                std::cout << "✅ Got position from LOGGER (timestamp: " << current.timestamp << ")" << std::endl;
                return true;
            } else {
                std::cout << "⚠️  LOGGER has no valid position data yet" << std::endl;
                return false;
            }
            
        } catch (const std::exception& e) {
            std::cout << "❌ Failed to get logger position: " << e.what() << std::endl;
            return false;
        }
#else
        // Simulation mode
        pos = Position(13.79, 138.2, 120.4, "Simulated Logger");
        return true;
#endif
    }
    
    bool getCurrentJoints() {
        if (!connected) {
            std::cout << "❌ Robot not connected!" << std::endl;
            return false;
        }
        
#ifdef USE_ABB
        try {
            // Get from logger
            RobotPosition current = robot->getCurrentPosition();
            if (current.valid) {
                std::cout << "🦾 Current joints (Logger): J1=" << std::fixed << std::setprecision(2)
                          << current.joint1 << " J2=" << current.joint2 << " J3=" << current.joint3
                          << " J4=" << current.joint4 << " J5=" << current.joint5 << " J6=" << current.joint6 << std::endl;
                return true;
            }
            
            // Fallback to query
            double j1, j2, j3, j4, j5, j6;
            if (robot->queryJointPositions(j1, j2, j3, j4, j5, j6)) {
                std::cout << "🦾 Current joints (Query): J1=" << std::fixed << std::setprecision(2)
                          << j1 << " J2=" << j2 << " J3=" << j3
                          << " J4=" << j4 << " J5=" << j5 << " J6=" << j6 << std::endl;
                return true;
            }
            
            return false;
            
        } catch (const std::exception& e) {
            std::cout << "❌ Failed to get joint positions: " << e.what() << std::endl;
            return false;
        }
#else
        std::cout << "🦾 Simulated joints: J1=0.0 J2=-30.0 J3=45.0 J4=0.0 J5=15.0 J6=0.0" << std::endl;
        return true;
#endif
    }
    
    void startPositionMonitoring() {
        position_monitoring.store(true);
        std::cout << "📊 Started real-time position monitoring (press any key to stop)" << std::endl;
    }
    
    void stopPositionMonitoring() {
        if (position_monitoring.load()) {
            position_monitoring.store(false);
            std::cout << "\n📊 Stopped position monitoring" << std::endl;
        }
    }
    
    bool testVacuum() {
        if (!connected) {
            std::cout << "❌ Robot not connected!" << std::endl;
            return false;
        }
        
#ifdef USE_ABB
        try {
            if (!robot->setVacuum(true)) {
                std::cout << "❌ Failed to turn vacuum ON" << std::endl;
                return false;
            }
            std::cout << "✅ Vacuum ON" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            if (!robot->setVacuum(false)) {
                std::cout << "❌ Failed to turn vacuum OFF" << std::endl;
                return false;
            }
            std::cout << "✅ Vacuum OFF" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cout << "❌ Vacuum test failed: " << e.what() << std::endl;
            return false;
        }
#else
        std::cout << "🎭 SIMULATION: Vacuum ON/OFF cycle completed" << std::endl;
        return true;
#endif
    }
    
    void emergencyStop() {
        std::cout << "🛑 EMERGENCY STOP!" << std::endl;
        
#ifdef USE_ABB
        if (robot && connected) {
            try {
                // ABB doesn't have built-in emergency stop in open_abb protocol
                // But we can send a close connection command to stop communication
                robot->disconnect();
                connected = false;
                std::cout << "✅ Connection terminated - robot should stop" << std::endl;
                std::cout << "⚠️  Please use physical emergency stop on robot if needed!" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "❌ Emergency stop failed: " << e.what() << std::endl;
            }
        }
#else
        std::cout << "🎭 SIMULATION: Emergency stop executed" << std::endl;
#endif
    }
    
    void runBasicTest() {
        std::cout << "\n🧪 Running Basic Robot Test Sequence with Logger" << std::endl;
        std::cout << "=================================================" << std::endl;
        
        // Define test positions (adjust according to your robot workspace)
        // Using identity quaternion (no rotation) - [1, 0, 0, 0]
        std::vector<Position> testPositions = {
            Position(0.3, 0.0, 0.2, "Home"),
            Position(0.4, 0.0, 0.2, "Right"),
            Position(0.3, 0.1, 0.2, "Forward"),
            Position(0.2, 0.0, 0.2, "Left"),
            Position(0.3, -0.1, 0.2, "Back"),
            Position(0.3, 0.0, 0.25, "Up"),
            Position(0.3, 0.0, 0.15, "Down"),
            Position(0.3, 0.0, 0.2, "Home Final")
        };
        
        // Start position monitoring
        startPositionMonitoring();
        
        for (size_t i = 0; i < testPositions.size(); ++i) {
            std::cout << "\n📍 Step " << (i + 1) << "/" << testPositions.size() << ": ";
            
            if (!moveToPosition(testPositions[i], 50.0, 30.0)) {  // Slower for testing
                std::cout << "❌ Test failed at step " << (i + 1) << std::endl;
                stopPositionMonitoring();
                return;
            }
            
            std::cout << "⏳ Waiting 3 seconds before next move..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        
        stopPositionMonitoring();
        std::cout << "\n✅ Basic test sequence completed successfully!" << std::endl;
    }
};

void printMenu() {
    std::cout << "\n🤖 ABB Robot Test Menu (with Logger Support):" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "1. Connect to robot" << std::endl;
    std::cout << "2. Disconnect from robot" << std::endl;
    std::cout << "3. Get current position (Logger + SERVER fallback)" << std::endl;
    std::cout << "4. Get current joint positions" << std::endl;
    std::cout << "5. Get LOGGER position only" << std::endl;
    std::cout << "6. Move to home position" << std::endl;
    std::cout << "7. Move to custom position" << std::endl;
    std::cout << "8. Run basic test sequence" << std::endl;
    std::cout << "9. Start/Stop position monitoring" << std::endl;
    std::cout << "10. Test vacuum system" << std::endl;
    std::cout << "11. Emergency stop" << std::endl;
    std::cout << "12. Check connection status" << std::endl;
    std::cout << "13. Exit" << std::endl;
    std::cout << "Enter choice (1-13): ";
}

int main() {
    std::cout << "🤖 Enhanced ABB Robot Test Program with Logger Support" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "Target robot: " << ROBOT_IP << std::endl;
    std::cout << "Motion port: " << MOTION_PORT << " (SERVER.mod)" << std::endl;
    std::cout << "Logger port: " << LOGGER_PORT << " (LOGGER.mod)" << std::endl;
    
#ifdef USE_ABB
    std::cout << "🔧 ABB support: ENABLED" << std::endl;
#else
    std::cout << "⚠️  ABB support: DISABLED (Simulation mode)" << std::endl;
#endif

    RobotTestWithLogger robotTest;
    int choice;
    static bool monitoring = false;
    
    while (true) {
        printMenu();
        std::cin >> choice;
        
        switch (choice) {
            case 1: {
                std::cout << "\n🔌 Connecting to robot..." << std::endl;
                if (robotTest.connect()) {
                    std::cout << "🎉 Connection successful!" << std::endl;
                } else {
                    std::cout << "💥 Connection failed!" << std::endl;
                }
                break;
            }
            
            case 2: {
                std::cout << "\n🔌 Disconnecting..." << std::endl;
                robotTest.disconnect();
                monitoring = false;
                break;
            }
            
            case 3: {
                std::cout << "\n📍 Getting current Cartesian position..." << std::endl;
                Position currentPos;
                if (robotTest.getCurrentPosition(currentPos)) {
                    std::cout << "✅ ";
                    currentPos.print();
                    std::cout << std::endl;
                } else {
                    std::cout << "❌ Failed to get current position" << std::endl;
                }
                break;
            }
            
            case 4: {
                std::cout << "\n🦾 Getting current joint positions..." << std::endl;
                robotTest.getCurrentJoints();
                break;
            }
            
            case 5: {
                std::cout << "\n📊 Getting LOGGER position only..." << std::endl;
                Position loggerPos;
                if (robotTest.getLoggerPositionOnly(loggerPos)) {
                    std::cout << "✅ ";
                    loggerPos.print();
                    std::cout << std::endl;
                } else {
                    std::cout << "❌ No valid LOGGER data available" << std::endl;
                }
                break;
            }
            
            case 6: {
                std::cout << "\n🏠 Moving to home position..." << std::endl;
                Position home(0.3, 0.0, 0.2, "Home");
                robotTest.moveToPosition(home, 100.0, 50.0);
                break;
            }
            
            case 7: {
                std::cout << "\n🎯 Enter custom position:" << std::endl;
                double x, y, z, q0, qx, qy, qz, tcp_speed, ori_speed;
                std::cout << "X (m): "; std::cin >> x;
                std::cout << "Y (m): "; std::cin >> y;
                std::cout << "Z (m): "; std::cin >> z;
                std::cout << "Q0 (w): "; std::cin >> q0;
                std::cout << "QX (x): "; std::cin >> qx;
                std::cout << "QY (y): "; std::cin >> qy;
                std::cout << "QZ (z): "; std::cin >> qz;
                std::cout << "TCP Speed (mm/s): "; std::cin >> tcp_speed;
                std::cout << "Ori Speed (deg/s): "; std::cin >> ori_speed;
                
                Position custom(x, y, z, q0, qx, qy, qz, "Custom");
                robotTest.moveToPosition(custom, tcp_speed, ori_speed);
                break;
            }
            
            case 8: {
                if (!robotTest.isConnected()) {
                    std::cout << "❌ Please connect to robot first!" << std::endl;
                } else {
                    robotTest.runBasicTest();
                }
                break;
            }
            
            case 9: {
                if (!robotTest.isConnected()) {
                    std::cout << "❌ Please connect to robot first!" << std::endl;
                } else if (!monitoring) {
                    robotTest.startPositionMonitoring();
                    monitoring = true;
                    std::cout << "Press Enter to stop monitoring...";
                    std::cin.ignore();
                    std::cin.get();
                    robotTest.stopPositionMonitoring();
                    monitoring = false;
                } else {
                    robotTest.stopPositionMonitoring();
                    monitoring = false;
                }
                break;
            }
            
            case 10: {
                if (!robotTest.isConnected()) {
                    std::cout << "❌ Please connect to robot first!" << std::endl;
                } else {
                    std::cout << "\n🔧 Testing vacuum system..." << std::endl;
                    robotTest.testVacuum();
                }
                break;
            }
            
            case 11: {
                robotTest.emergencyStop();
                monitoring = false;
                break;
            }
            
            case 12: {
                std::cout << "\n📡 Connection status: " 
                          << (robotTest.isConnected() ? "✅ CONNECTED" : "❌ DISCONNECTED") 
                          << std::endl;
                break;
            }
            
            case 13: {
                std::cout << "\n👋 Exiting program..." << std::endl;
                if (robotTest.isConnected()) {
                    std::cout << "🔌 Disconnecting robot..." << std::endl;
                    robotTest.disconnect();
                }
                std::cout << "✅ Program terminated safely" << std::endl;
                return 0;
            }
            
            default: {
                std::cout << "❌ Invalid choice! Please enter 1-13." << std::endl;
                break;
            }
        }
        
        if (choice != 9) {  // Skip "press enter" for monitoring mode
            std::cout << "\nPress Enter to continue...";
            std::cin.ignore();
            std::cin.get();
        }
    }
    
    return 0;
}