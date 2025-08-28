#ifndef ABB_COMM_H
#define ABB_COMM_H

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using std::string;

/**
 * Structure to hold robot position data
 */
struct RobotPosition {
    // Cartesian position
    double x, y, z;
    double q0, qx, qy, qz;  // Quaternion orientation
    
    // Joint positions
    double joint1, joint2, joint3, joint4, joint5, joint6;
    
    // External axes (if available)
    double eax_a, eax_b, eax_c, eax_d, eax_e, eax_f;
    
    // Timestamp
    long long timestamp;
    bool valid;
    
    RobotPosition() : x(0), y(0), z(0), q0(1), qx(0), qy(0), qz(0),
                     joint1(0), joint2(0), joint3(0), joint4(0), joint5(0), joint6(0),
                     eax_a(0), eax_b(0), eax_c(0), eax_d(0), eax_e(0), eax_f(0),
                     timestamp(0), valid(false) {}
};

/**
 * ABB Robot Communication Class
 */
namespace abb_comm
{
    // Original command formatting functions
    string pingRobot(int idCode = 0);
    string setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode = 0);
    string setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode = 0);
    string getCartesian(int idCode = 0);
    string getJoints(int idCode = 0);
    string setTool(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode = 0);
    string setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode = 0);
    string setSpeed(double tcp, double ori, int idCode = 0);
    string setZone(bool fine = 0, double tcp_mm = 5.0, double ori_mm = 5.0, double ori_deg = 1.0, int idCode = 0);
    string specialCommand(int command, double param1, double param2, double param3, double param4, double param5, int idCode = 0);
    string setVacuum(int vacuum = 0, int idCode = 0);
    string setDIO(int dio_number = 0, int dio_state = 0, int idCode = 0);
    string closeConnection(int idCode = 0);
    
    // Response parsing functions
    int parseCartesian(string msg, double *x, double *y, double *z,
                      double *q0, double *qx, double *qy, double *qz);
    int parseJoints(string msg, double *joint1, double *joint2, 
                   double *joint3, double *joint4, double *joint5, double *joint6);
}

/**
 * ABB Robot Controller Class - handles both motion commands and position logging
 */
class AbbRobotController {
private:
    // Motion server connection (SERVER.mod - port 5000)
    int motion_socket;
    string robot_ip;
    int motion_port;
    
    // Logger connection (LOGGER.mod - port 5001)
    int logger_socket;
    int logger_port;
    std::thread logger_thread;
    std::atomic<bool> logger_running;
    
    // Position data
    RobotPosition current_position;
    std::mutex position_mutex;
    
    // Callback for position updates
    std::function<void(const RobotPosition&)> position_callback;
    
    // Internal methods
    bool connectToMotionServer();
    bool connectToLoggerServer();
    void loggerWorker();
    bool parseLoggerMessage(const string& message, RobotPosition& pos);
    
public:
    AbbRobotController(const string& ip = "192.168.125.1", 
                      int motion_port = 5000, 
                      int logger_port = 5001);
    ~AbbRobotController();
    
    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // Motion commands
    bool sendCommand(const string& command);
    string sendCommandWithResponse(const string& command);
    
    // Convenience methods for common commands
    bool moveCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode = 0);
    bool moveJoints(double j1, double j2, double j3, double j4, double j5, double j6, int idCode = 0);
    bool setSpeed(double tcp, double ori, int idCode = 0);
    bool setVacuum(bool on, int idCode = 0);
    bool ping(int idCode = 0);
    
    // Position monitoring
    bool startPositionLogging();
    void stopPositionLogging();
    RobotPosition getCurrentPosition();
    void setPositionCallback(std::function<void(const RobotPosition&)> callback);
    
    // Query current position from motion server (blocking)
    bool queryCartesianPosition(double& x, double& y, double& z, double& q0, double& qx, double& qy, double& qz);
    bool queryJointPositions(double& j1, double& j2, double& j3, double& j4, double& j5, double& j6);
};

#endif // ABB_COMM_H