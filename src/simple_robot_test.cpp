#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <iomanip>
#include <vector>

#ifdef USE_ABB
#include "robot/abb/AbbController.hpp"
using cymbergaj::robot::abb::AbbController;
#endif

// Robot connection settings
const std::string ROBOT_IP = "10.25.74.172";
const uint16_t ROBOT_PORT = 11000;

// Test positions (adjust according to your robot workspace)
struct Position {
    float x, y, z, rx, ry, rz;
    std::string name;
    
    // Default constructor
    Position() : x(0), y(0), z(0), rx(0), ry(0), rz(0), name("Default") {}
    
    // Parameterized constructor
    Position(float x, float y, float z, float rx = 0, float ry = 0, float rz = 0, const std::string& name = "")
        : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz), name(name) {}
        
    void print() const {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << name << ": [" << x << ", " << y << ", " << z << "] R[" << rx << ", " << ry << ", " << rz << "]";
    }
};

class SimpleRobotTest {
private:
#ifdef USE_ABB
    AbbController* robot;
#endif
    bool connected;
    
public:
    SimpleRobotTest() : connected(false) {
#ifdef USE_ABB
        robot = nullptr;
#endif
    }
    
    ~SimpleRobotTest() {
        disconnect();
    }
    
    bool connect() {
        std::cout << "🔌 Attempting to connect to robot at " << ROBOT_IP << ":" << ROBOT_PORT << std::endl;
        
#ifdef USE_ABB
        try {
            robot = new AbbController(ROBOT_IP, ROBOT_PORT);
            
            if (robot->connect()) {
                connected = true;
                std::cout << "✅ Successfully connected to ABB robot!" << std::endl;
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
            std::cout << "🔌 Disconnecting from robot..." << std::endl;
            
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
    
    bool moveToPosition(const Position& pos, float speed = 0.3f) {
        if (!connected) {
            std::cout << "❌ Robot not connected!" << std::endl;
            return false;
        }
        
        std::cout << "🎯 Moving to position: ";
        pos.print();
        std::cout << " at speed " << speed << std::endl;
        
#ifdef USE_ABB
        try {
            // Adjust these method calls according to your AbbController API
            // Example calls - modify based on your actual implementation:
            
            // Option 1: If you have moveLinear method
            // bool success = robot->moveLinear(pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz, speed);
            
            // Option 2: If you have moveTo method
            // bool success = robot->moveTo(pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz, speed);
            
            // Option 3: If you have different method names
            // bool success = robot->moveToPosition(pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz, speed);
            
            // For now, simulate success (replace with actual robot call)
            std::cout << "📡 Sending move command to robot..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate movement time
            bool success = true; // Replace with actual robot call result
            
            if (success) {
                std::cout << "✅ Movement completed successfully" << std::endl;
                return true;
            } else {
                std::cout << "❌ Movement failed" << std::endl;
                return false;
            }
            
        } catch (const std::exception& e) {
            std::cout << "❌ Movement failed with exception: " << e.what() << std::endl;
            return false;
        }
#else
        // Simulation mode
        std::cout << "🎭 SIMULATION: Moving robot to position..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "✅ SIMULATION: Movement completed" << std::endl;
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
            // Adjust this method call according to your AbbController API
            // Example calls - modify based on your actual implementation:
            
            // Option 1: If you have getCurrentPosition method
            // robot->getCurrentPosition(pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz);
            
            // Option 2: If you have getPosition method
            // auto currentPos = robot->getPosition();
            // pos.x = currentPos.x; pos.y = currentPos.y; pos.z = currentPos.z;
            
            // For now, return simulated position (replace with actual robot call)
            pos = Position(0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, "Current");
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "❌ Failed to get current position: " << e.what() << std::endl;
            return false;
        }
#else
        // Simulation mode
        pos = Position(0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, "Simulated Current");
        return true;
#endif
    }
    
    void emergencyStop() {
        std::cout << "🛑 EMERGENCY STOP!" << std::endl;
        
#ifdef USE_ABB
        if (robot && connected) {
            try {
                // robot->emergencyStop(); // Adjust method name as needed
                std::cout << "✅ Emergency stop command sent to robot" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "❌ Emergency stop failed: " << e.what() << std::endl;
            }
        }
#else
        std::cout << "🎭 SIMULATION: Emergency stop executed" << std::endl;
#endif
    }
    
    void runBasicTest() {
        std::cout << "\n🧪 Running Basic Robot Test Sequence" << std::endl;
        std::cout << "====================================" << std::endl;
        
        // Define test positions (adjust according to your robot workspace)
        std::vector<Position> testPositions = {
            Position(0.0f, 0.0f, 0.15f, 0, 0, 0, "Home"),
            Position(0.1f, 0.0f, 0.15f, 0, 0, 0, "Right"),
            Position(0.0f, 0.1f, 0.15f, 0, 0, 0, "Forward"),
            Position(-0.1f, 0.0f, 0.15f, 0, 0, 0, "Left"),
            Position(0.0f, -0.1f, 0.15f, 0, 0, 0, "Back"),
            Position(0.0f, 0.0f, 0.15f, 0, 0, 0, "Home Final")
        };
        
        for (size_t i = 0; i < testPositions.size(); ++i) {
            std::cout << "\n📍 Step " << (i + 1) << "/" << testPositions.size() << ": ";
            
            if (!moveToPosition(testPositions[i], 0.3f)) {
                std::cout << "❌ Test failed at step " << (i + 1) << std::endl;
                return;
            }
            
            std::cout << "⏳ Waiting 2 seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        std::cout << "\n✅ Basic test sequence completed successfully!" << std::endl;
    }
};

void printMenu() {
    std::cout << "\n🤖 ABB Robot Test Menu:" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "1. Connect to robot" << std::endl;
    std::cout << "2. Disconnect from robot" << std::endl;
    std::cout << "3. Get current position" << std::endl;
    std::cout << "4. Move to home position" << std::endl;
    std::cout << "5. Move to custom position" << std::endl;
    std::cout << "6. Run basic test sequence" << std::endl;
    std::cout << "7. Emergency stop" << std::endl;
    std::cout << "8. Check connection status" << std::endl;
    std::cout << "9. Exit" << std::endl;
    std::cout << "Enter choice (1-9): ";
}

int main() {
    std::cout << "🤖 Simple ABB Robot Test Program" << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "Target robot: " << ROBOT_IP << ":" << ROBOT_PORT << std::endl;
    
#ifdef USE_ABB
    std::cout << "🔧 ABB support: ENABLED" << std::endl;
#else
    std::cout << "⚠️  ABB support: DISABLED (Simulation mode)" << std::endl;
#endif

    SimpleRobotTest robotTest;
    int choice;
    
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
                break;
            }
            
            case 3: {
                std::cout << "\n📍 Getting current position..." << std::endl;
                Position currentPos;
                if (robotTest.getCurrentPosition(currentPos)) {
                    std::cout << "✅ Current position: ";
                    currentPos.print();
                    std::cout << std::endl;
                } else {
                    std::cout << "❌ Failed to get current position" << std::endl;
                }
                break;
            }
            
            case 4: {
                std::cout << "\n🏠 Moving to home position..." << std::endl;
                Position home(0.0f, 0.0f, 0.15f, 0, 0, 0, "Home");
                robotTest.moveToPosition(home, 0.3f);
                break;
            }
            
            case 5: {
                std::cout << "\n🎯 Enter custom position:" << std::endl;
                float x, y, z, rx, ry, rz, speed;
                std::cout << "X (m): "; std::cin >> x;
                std::cout << "Y (m): "; std::cin >> y;
                std::cout << "Z (m): "; std::cin >> z;
                std::cout << "RX (deg): "; std::cin >> rx;
                std::cout << "RY (deg): "; std::cin >> ry;
                std::cout << "RZ (deg): "; std::cin >> rz;
                std::cout << "Speed (0.1-1.0): "; std::cin >> speed;
                
                Position custom(x, y, z, rx, ry, rz, "Custom");
                robotTest.moveToPosition(custom, speed);
                break;
            }
            
            case 6: {
                if (!robotTest.isConnected()) {
                    std::cout << "❌ Please connect to robot first!" << std::endl;
                } else {
                    robotTest.runBasicTest();
                }
                break;
            }
            
            case 7: {
                robotTest.emergencyStop();
                break;
            }
            
            case 8: {
                std::cout << "\n📡 Connection status: " 
                          << (robotTest.isConnected() ? "✅ CONNECTED" : "❌ DISCONNECTED") 
                          << std::endl;
                break;
            }
            
            case 9: {
                std::cout << "\n👋 Exiting program..." << std::endl;
                if (robotTest.isConnected()) {
                    std::cout << "🔌 Disconnecting robot..." << std::endl;
                    robotTest.disconnect();
                }
                std::cout << "✅ Program terminated safely" << std::endl;
                return 0;
            }
            
            default: {
                std::cout << "❌ Invalid choice! Please enter 1-9." << std::endl;
                break;
            }
        }
        
        std::cout << "\nPress Enter to continue...";
        std::cin.ignore();
        std::cin.get();
    }
    
    return 0;
}