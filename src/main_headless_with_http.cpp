//main_headless_with_http.cpp
// Fast headless tracker with HTTP interface and ABB robot control
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <getopt.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <array>

#include "capture.hpp"
#include "detector.hpp"
#include "tracker.hpp"
#include "utils.hpp"
#include "lock_free_queue.hpp"

#ifdef USE_ABB
#include "robot/abb/AbbController.hpp"  // Use the correct ABB controller
using cymbergaj::robot::abb::AbbController;

struct RobotPosition {
    double x{0}, y{0}, z{0}, q0{1}, qx{0}, qy{0}, qz{0};
    bool valid{false};
};
#else
// Mock types when ABB is disabled
class AbbController {
public:
    AbbController(const std::string&, uint16_t) {}
    bool connect() { return false; }
    void disconnect() {}
    bool isConnected() const { return false; }
    bool setSpeed(double, double) { return false; }
    bool moveL(const std::array<double,7>&) { return false; }
    bool getCartesian(std::array<double,7>&) { return false; }
    const std::string& lastError() const { static std::string err = "Mock controller"; return err; }
};

struct RobotPosition {
    double x{0}, y{0}, z{0}, q0{1}, qx{0}, qy{0}, qz{0};
    bool valid{false};
};
#endif

using namespace std::chrono;

std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nShutting down..." << std::endl;
        g_running = false;
    }
}

struct Config {
    int width = 640;
    int height = 480;
    int fps = 60;
    std::string backend = "v4l2";
    std::string method = "color";
    std::string config_file = "config/config.yaml";
    int queue_size = 5;
    bool enable_profiling = true;
    bool save_output = false;
    std::string output_file = "tracking_data.csv";
    int http_port = 8080;
    
    // Missing options for compatibility
    std::string pipeline = "";
    bool show_window = true;
    bool enable_stream = false;
    std::string stream_address = "";
};

// ======= Robot Control Structures =======
struct Position3D {
    float x, y, z;
    float q0, qx, qy, qz; // Quaternion (w, x, y, z)
    
    Position3D(float x = 0, float y = 0, float z = 0, float q0 = 1, float qx = 0, float qy = 0, float qz = 0)
        : x(x), y(y), z(z), q0(q0), qx(qx), qy(qy), qz(qz) {}
        
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "(" << x << ", " << y << ", " << z << ") Q(" << q0 << ", " << qx << ", " << qy << ", " << qz << ")";
        return oss.str();
    }
};

struct RobotCommand {
    // UNITS: meters for positions, unitless 0.05..1.0 for speed knob
    float x{0}, y{0}, z{0};
    float q0{1}, qx{0}, qy{0}, qz{0};
    float speed{0.5f};
    bool execute_strike{false};
    std::string name;
};

struct GameStrategy {
    enum Mode { DEFENSIVE, OFFENSIVE, INTERCEPT };
    Mode current_mode = DEFENSIVE;
    cv::Point2f robot_position = cv::Point2f(0, 0); // m, in table coordinates
    float strike_threshold_m = 0.05f; // 5 cm
};

struct RobotStatus {
    Position3D current_position;
    Position3D target_position;
    bool is_moving = false;
    bool is_connected = false;
    std::string last_error;
    std::string current_test;
    std::chrono::high_resolution_clock::time_point last_update;
    
    RobotStatus() {
        last_update = std::chrono::high_resolution_clock::now();
    }
};

struct TableCalibration {
    std::vector<cv::Point2f> img_pts;   // 4 corner points in pixels
    float width_mm  = 1000.0f;
    float height_mm = 500.0f;
    int origin_index = 0;
    bool clockwise = true;
    
    cv::Mat H_pix2tab;
    cv::Mat H_tab2pix;
    bool ready = false;

    std::vector<cv::Point2f> table_corners_mm() const {
        cv::Point2f P0(0.0f, 0.0f);
        cv::Point2f P1(width_mm, 0.0f);
        cv::Point2f P2(width_mm, height_mm);
        cv::Point2f P3(0.0f, height_mm);
        std::vector<cv::Point2f> base = {P0,P1,P2,P3};
        std::vector<cv::Point2f> out(4);
        for (int i=0;i<4;++i) {
            int idx = clockwise ? (origin_index + i) % 4
                                : (origin_index - i + 4) % 4;
            out[i] = base[idx];
        }
        return out;
    }

    void computeHomography() {
        ready = false;
        if (img_pts.size() != 4) return;
        std::vector<cv::Point2f> dst = table_corners_mm();
        H_pix2tab = cv::findHomography(img_pts, dst);
        if (!H_pix2tab.empty()) {
            H_tab2pix = H_pix2tab.inv();
            ready = true;
        }
    }

    cv::Point2f pixelToTableMeters(const cv::Point2f& p) const {
        if (!ready) return {0.f, 0.f};
        std::vector<cv::Point2f> src{p}, dst;
        cv::perspectiveTransform(src, dst, H_pix2tab);
        return { dst[0].x / 1000.f, dst[0].y / 1000.f };
    }
};

// Shared data between detection and HTTP threads
struct SharedData {
    std::mutex mutex;
    
    // Current detection settings
    DetectionMethod detection_method = DetectionMethod::COLOR_THRESHOLD;
    cv::Scalar hsv_lower = cv::Scalar(0, 100, 100);
    cv::Scalar hsv_upper = cv::Scalar(10, 255, 255);
    int min_area = 100;
    int max_area = 5000;
    float min_circularity = 0.7f;
    cv::Rect table_roi = cv::Rect(0, 0, 640, 480);
    
    // Current frame and detection results
    cv::Mat current_frame;
    cv::Mat detection_mask;
    cv::Point2f puck_position;
    cv::Point2f puck_velocity;
    bool puck_detected = false;
    float puck_confidence = 0.0f;
    uint64_t frame_count = 0;
    
    // Stats
    float fps = 0.0f;
    float latency_ms = 0.0f;
    int dropped_frames = 0;
} g_shared;

// Global robot state
GameStrategy g_strategy;
RobotStatus g_robot_status;
TableCalibration g_table_calib;
std::mutex g_robot_mutex;

struct RobotConnection {
    std::string ip = "10.25.74.172";
    uint16_t port = 5000;
} g_connection;

#ifdef USE_ABB
static AbbController* g_robot = nullptr;
#else
static AbbController* g_robot = nullptr; // mock when USE_ABB=OFF
#endif

Config parse_args(int argc, char* argv[]) {
    Config cfg;
    
    static struct option long_options[] = {
        {"width", required_argument, 0, 'w'},
        {"height", required_argument, 0, 'h'},
        {"fps", required_argument, 0, 'f'},
        {"backend", required_argument, 0, 'b'},
        {"pipeline", required_argument, 0, 'p'},
        {"method", required_argument, 0, 'm'},
        {"config", required_argument, 0, 'c'},
        {"no-window", no_argument, 0, 'n'},
        {"stream", required_argument, 0, 's'},
        {"queue-size", required_argument, 0, 'q'},
        {"http-port", required_argument, 0, 'P'},
        {"save-output", no_argument, 0, 'o'},
        {"help", no_argument, 0, '?'},
        {0, 0, 0, 0}
    };
    
    int opt;
    int option_index = 0;
    
    while ((opt = getopt_long(argc, argv, "w:h:f:b:p:m:c:ns:q:P:o?", 
                              long_options, &option_index)) != -1) {
        switch (opt) {
            case 'w': cfg.width = std::stoi(optarg); break;
            case 'h': cfg.height = std::stoi(optarg); break;
            case 'f': cfg.fps = std::stoi(optarg); break;
            case 'b': cfg.backend = optarg; break;
            case 'p': cfg.pipeline = optarg; break;
            case 'm': cfg.method = optarg; break;
            case 'c': cfg.config_file = optarg; break;
            case 'n': cfg.show_window = false; break;
            case 's': 
                cfg.enable_stream = true; 
                cfg.stream_address = optarg; 
                break;
            case 'q': cfg.queue_size = std::stoi(optarg); break;
            case 'P': cfg.http_port = std::stoi(optarg); break;
            case 'o': cfg.save_output = true; break;
            case '?':
                std::cout << "Usage: " << argv[0] << " [options]\n"
                         << "Options:\n"
                         << "  -w, --width WIDTH       Frame width (default: 640)\n"
                         << "  -h, --height HEIGHT     Frame height (default: 480)\n"
                         << "  -f, --fps FPS           Target FPS (default: 60)\n"
                         << "  -b, --backend BACKEND   Capture backend (default: v4l2)\n"
                         << "  -m, --method METHOD     Detection method: color|bgsub (default: color)\n"
                         << "  -c, --config FILE       Config file path (default: config/config.yaml)\n"
                         << "  -q, --queue-size SIZE   Frame queue size (default: 5)\n"
                         << "  -P, --http-port PORT    HTTP server port (default: 8080)\n"
                         << "  -o, --save-output       Save tracking data to CSV\n"
                         << "  --help                  Show this help\n";
                exit(0);
        }
    }
    
    return cfg;
}

// ======= Robot Control Functions =======
// Fixed calculateRobotCommand function with proper coordinate conversion and rate limiting
RobotCommand calculateRobotCommand(const cv::Point2f& puck_px, const cv::Point2f& velocity_px) {
    RobotCommand cmd{};
    cmd.speed = 0.5f; // 0.05..1.0 (later scaled to mm/s)
    cmd.z = 0.050f;   // 50mm above table in meters
    cmd.q0 = 1.0f;
    
    // Rate limiting - don't send commands too frequently
    static std::chrono::high_resolution_clock::time_point last_command_time;
    static const std::chrono::milliseconds MIN_COMMAND_INTERVAL(100); // 10Hz max
    
    auto now = std::chrono::high_resolution_clock::now();
    if (now - last_command_time < MIN_COMMAND_INTERVAL) {
        return cmd; // Return default command (no move)
    }
    last_command_time = now;

    // Convert to meters using proper scaling
    cv::Point2f puck_m, pred_m;
    
    if (g_table_calib.ready) {
        // Use homography transformation
        cv::Point2f table_coords = g_table_calib.pixelToTableMeters(puck_px);
        puck_m = table_coords;
        
        // Predict future position
        cv::Point2f pred_px = puck_px + velocity_px * 0.2f; // 200ms prediction
        pred_m = g_table_calib.pixelToTableMeters(pred_px);
    } else {
        // Fallback: Assume table dimensions ~0.4 x 0.3 m in ROI
        float table_width_m = 0.400f;
        float table_height_m = 0.300f;
        float roi_width = g_shared.table_roi.width;
        float roi_height = g_shared.table_roi.height;
        
        float center_x = g_shared.table_roi.x + roi_width / 2.0f;
        float center_y = g_shared.table_roi.y + roi_height / 2.0f;
        
        puck_m.x = ((puck_px.x - center_x) / roi_width) * table_width_m;
        puck_m.y = ((puck_px.y - center_y) / roi_height) * table_height_m;
        
        cv::Point2f pred_px = puck_px + velocity_px * 0.2f;
        pred_m.x = ((pred_px.x - center_x) / roi_width) * table_width_m;
        pred_m.y = ((pred_px.y - center_y) / roi_height) * table_height_m;
    }

    // Robot position in meters (track actual robot position)
    cv::Point2f robot_m = g_strategy.robot_position;
    float distance_m = cv::norm(puck_m - robot_m);
    
    // Velocity threshold for moving puck
    float puck_speed = cv::norm(velocity_px);
    bool puck_is_moving = puck_speed > 5.0f; // pixels/s threshold

    switch (g_strategy.current_mode) {
        case GameStrategy::DEFENSIVE:
            // Stay on defensive line, match Y coordinate
            cmd.x = -0.100f; // Stay back 100mm from center
            cmd.y = std::clamp(puck_m.y * 0.5f, -0.120f, 0.120f);
            cmd.name = "Defensive";
            break;
            
        case GameStrategy::OFFENSIVE:
            // Move towards predicted puck position
            cmd.x = std::clamp(pred_m.x + 0.030f, -0.180f, 0.180f);
            cmd.y = std::clamp(pred_m.y, -0.120f, 0.120f);
            cmd.name = "Offensive";
            break;
            
        case GameStrategy::INTERCEPT:
            // Only intercept if puck is moving towards us (screen x+ means right)
            if (puck_is_moving && velocity_px.x > 0) {
                cmd.x = std::clamp(pred_m.x, -0.180f, 0.180f);
                cmd.y = std::clamp(pred_m.y, -0.120f, 0.120f);
                cmd.name = "Intercept";
                
                // Strike only if very close and aligned
                if (distance_m < 0.030f && std::abs(puck_m.y - robot_m.y) < 0.020f) {
                    cmd.execute_strike = true;
                    cmd.speed = 0.9f;
                    cmd.name = "Strike!";
                }
            } else {
                // Return to defensive position if puck not approaching
                cmd.x = -0.050f;
                cmd.y = std::clamp(puck_m.y * 0.3f, -0.100f, 0.100f);
                cmd.name = "Wait";
            }
            break;
    }

    // Enhanced safety limits (robot workspace) in meters
    cmd.x = std::clamp(cmd.x, -0.200f, 0.200f);
    cmd.y = std::clamp(cmd.y, -0.150f, 0.150f);
    cmd.z = std::clamp(cmd.z,  0.020f, 0.100f);

    // Debug output (reduced frequency)
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0) { // Every 10th command
        std::cout << "[ROBOT] Puck px(" << puck_px.x << "," << puck_px.y 
                  << ") -> m(" << puck_m.x << "," << puck_m.y << ")" << std::endl;
        std::cout << "[ROBOT] " << cmd.name << ": (" << cmd.x << "," << cmd.y << "," << cmd.z 
                  << ") Speed: " << cmd.speed << " Dist: " << distance_m << "m" << std::endl;
    }

    return cmd;
}

// Add this function to set up proper table calibration fallback
void setupDefaultTableCalibration() {
    if (!g_table_calib.ready) {
        g_table_calib.width_mm = 800.0f;   // 80cm stół
        g_table_calib.height_mm = 500.0f;  // 50cm stół
        
        // Domyślne narożniki (dostosuj do swojej kamery)
        g_table_calib.img_pts = {
            cv::Point2f(80, 60),    // Lewy górny
            cv::Point2f(560, 60),   // Prawy górny  
            cv::Point2f(560, 420),  // Prawy dolny
            cv::Point2f(80, 420)    // Lewy dolny
        };
        
        g_table_calib.origin_index = 0;
        g_table_calib.clockwise = true;
        g_table_calib.computeHomography();
        
        if (g_table_calib.ready) {
            std::cout << "[CALIB] Default table calibration applied (800x500mm)" << std::endl;
        }
    }
}

// Modified sendRobotCommand with better error handling
bool sendRobotCommand(const RobotCommand& cmd) {
#ifdef USE_ABB
    std::lock_guard<std::mutex> lk(g_robot_mutex);
    if (!g_robot || !g_robot->isConnected()) {
        std::cout << "[ROBOT] Not connected!\n";
        return false;
    }

    try {
        // Set speed (mm/s and deg/s)
        double tcp_mm_s  = std::clamp<double>(cmd.speed, 0.05, 1.0) * 500.0; // 25..500 mm/s
        double ori_deg_s = std::clamp<double>(cmd.speed, 0.05, 1.0) * 100.0; // 5..100 deg/s

        if (!g_robot->setSpeed(tcp_mm_s, ori_deg_s)) {
            std::cout << "[ROBOT] Failed to set speed\n";
            return false;
        }

        // Quaternion for tool (adjust if needed for your setup)
        double q0 = cmd.q0, qx = cmd.qx, qy = cmd.qy, qz = cmd.qz;
        
        // Positions already in meters
        double x_m = cmd.x;  
        double y_m = cmd.y;
        double z_m = cmd.z;
        
        // Safety limits (in meters)
        x_m = std::clamp(x_m, -0.600, 0.600);
        y_m = std::clamp(y_m, -0.400, 0.400);
        z_m = std::clamp(z_m,  0.020, 0.100);

        std::cout << "[ROBOT] Moving to: (" 
                  << x_m*1000.0 << ", " << y_m*1000.0 << ", " << z_m*1000.0 
                  << ") mm at speed " << tcp_mm_s << " mm/s\n";

        // Send the actual move command (linear)
        std::array<double, 7> pose = {x_m, y_m, z_m, q0, qx, qy, qz};
        bool success = g_robot->moveL(pose);
        
        if (!success) {
            std::cout << "[ROBOT] moveL failed\n";
            g_robot_status.last_error = "Move command failed";
            return false;
        }

        // Handle strike motion if requested
        if (cmd.execute_strike) {
            std::cout << "[ROBOT] Executing strike motion\n";
            
            // Quick forward motion for strike
            g_robot->setSpeed(800.0, 120.0);  // Fast speed for strike
            
            double strike_distance = 0.08; // 80mm strike distance
            double y_strike = y_m + strike_distance; // meters
            
            // Strike forward
            std::array<double,7> pose_strike = {x_m, y_strike, z_m, q0, qx, qy, qz};
            g_robot->moveL(pose_strike);
            
            // Brief pause
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Return to position
            g_robot->setSpeed(tcp_mm_s, ori_deg_s);
            g_robot->moveL(pose);
        }
        
        return true;
        
    } catch (const std::exception& e) {
        g_robot_status.last_error = std::string("Command failed: ") + e.what();
        std::cout << "[ROBOT] Exception: " << e.what() << "\n";
        return false;
    }
#else
    // Simulation mode
    std::cout << "[SIM] Robot Command: x=" << cmd.x << ", y=" << cmd.y << ", z=" << cmd.z
              << ", speed=" << cmd.speed << ", strike=" << (cmd.execute_strike?"yes":"no") << std::endl;
    return true;
#endif
}


void capture_thread(FrameCapture* capture, LockFreeQueue<FrameData>* queue, 
                   std::atomic<bool>& running, PerfStats* stats) {
    cv::Mat frame;
    uint64_t frame_count = 0;
    
    while (running) {
        auto start = high_resolution_clock::now();
        
        if (capture->grab(frame)) {
            FrameData data;
            data.frame = frame.clone();
            data.timestamp = duration_cast<microseconds>(
                high_resolution_clock::now().time_since_epoch()).count();
            data.frame_id = frame_count++;
            
            // Update shared frame for HTTP display
            {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                g_shared.current_frame = frame.clone();
                g_shared.frame_count = frame_count;
            }
            
            if (!queue->push(data)) {
                stats->dropped_frames++;
            }
            
            auto end = high_resolution_clock::now();
            stats->capture_time = duration_cast<microseconds>(end - start).count();
        }
    }
}

void process_thread(LockFreeQueue<FrameData>* input_queue,
                   Detector* detector, KalmanTracker* tracker,
                   std::atomic<bool>& running, PerfStats* stats,
                   std::atomic<int>& frames_processed,
                   std::ofstream* output_file = nullptr) {
    
    FrameData frame_data;
    cv::Mat hsv, mask;
    
    while (running) {
        if (input_queue->pop(frame_data)) {
            auto start = high_resolution_clock::now();
            
            // Get current detection settings
            DetectionMethod method;
            cv::Scalar hsv_lower, hsv_upper;
            int min_area, max_area;
            float min_circularity;
            cv::Rect roi;
            
            {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                method = g_shared.detection_method;
                hsv_lower = g_shared.hsv_lower;
                hsv_upper = g_shared.hsv_upper;
                min_area = g_shared.min_area;
                max_area = g_shared.max_area;
                min_circularity = g_shared.min_circularity;
                roi = g_shared.table_roi;
            }
            
            // Update detector with current settings
            detector->setMethod(method);
            
            cv::Point2f puck_pos;
            bool detected = detector->detect(frame_data.frame, puck_pos);
            
            // Generate detection mask for HTTP display
            if (method == DetectionMethod::COLOR_THRESHOLD) {
                cv::cvtColor(frame_data.frame(roi), hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, hsv_lower, hsv_upper, mask);
            }
            
            if (detected) {
                tracker->update(puck_pos, frame_data.timestamp);
            } else {
                tracker->predictOnly(frame_data.timestamp);
            }
            
            // Update shared detection results
            cv::Point2f velocity = tracker->getVelocity();
            {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                g_shared.puck_position = tracker->getPosition();
                g_shared.puck_velocity = velocity;
                g_shared.puck_detected = detected;
                g_shared.puck_confidence = detected ? 1.0f : 0.0f;
                
                if (!mask.empty()) {
                    g_shared.detection_mask = cv::Mat::zeros(frame_data.frame.size(), CV_8UC1);
                    mask.copyTo(g_shared.detection_mask(roi));
                }
            }
            
            // Send robot commands if table is calibrated and puck is detected
            if (g_table_calib.ready && detected) {
                RobotCommand cmd = calculateRobotCommand(puck_pos, velocity);
                sendRobotCommand(cmd);
            }
            
            // Save data to CSV
            if (output_file && output_file->is_open()) {
                auto pos = tracker->getPosition();
                auto vel = tracker->getVelocity();
                *output_file << frame_data.timestamp << ","
                           << pos.x << "," << pos.y << ","
                           << vel.x << "," << vel.y << ","
                           << detected << std::endl;
            }
            
            auto end = high_resolution_clock::now();
            stats->process_time = duration_cast<microseconds>(end - start).count();

            frames_processed++;
            
            // Status display (reduced frequency)
            if (frame_data.frame_id % 30 == 0) {
                auto pos = tracker->getPosition();
                auto vel = tracker->getVelocity();
                std::cout << "\rFrame: " << frame_data.frame_id 
                         << " | Pos: (" << std::fixed << std::setprecision(1) 
                         << pos.x << ", " << pos.y << ")"
                         << " | Vel: (" << vel.x << ", " << vel.y << ")"
                         << " | " << (detected ? "TRACKED" : "LOST   ")
                         << std::flush;
            }
        } else {
            std::this_thread::sleep_for(microseconds(100));
        }
    }
}

// ======= HTTP Helper Functions =======
void send_http_response(int client_socket, const std::string& content_type, const std::string& content) {
    std::string response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + std::to_string(content.length()) + "\r\n"
        "Connection: close\r\n"
        "\r\n" + content;
    send(client_socket, response.c_str(), response.length(), 0);
}

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
    bool connected = false;
#ifdef USE_ABB
    connected = (g_robot != nullptr && g_robot->isConnected());
#else
    connected = g_robot_status.is_connected; // Use status from mock
#endif
    
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    os << "{"
       << "\"connected\":" << (connected ? "true" : "false") << ","
       << "\"moving\":" << (g_robot_status.is_moving ? "true" : "false") << ","
       << "\"ip\":\"" << escape_json(g_connection.ip) << "\","
       << "\"port\":" << g_connection.port << ","
       << "\"current_position\":{"
       << "\"x\":" << g_robot_status.current_position.x << ","
       << "\"y\":" << g_robot_status.current_position.y << ","
       << "\"z\":" << g_robot_status.current_position.z << ","
       << "\"q0\":" << g_robot_status.current_position.q0 << ","
       << "\"qx\":" << g_robot_status.current_position.qx << ","
       << "\"qy\":" << g_robot_status.current_position.qy << ","
       << "\"qz\":" << g_robot_status.current_position.qz
       << "},"
       << "\"target_position\":{"
       << "\"x\":" << g_robot_status.target_position.x << ","
       << "\"y\":" << g_robot_status.target_position.y << ","
       << "\"z\":" << g_robot_status.target_position.z << ","
       << "\"q0\":" << g_robot_status.target_position.q0 << ","
       << "\"qx\":" << g_robot_status.target_position.qx << ","
       << "\"qy\":" << g_robot_status.target_position.qy << ","
       << "\"qz\":" << g_robot_status.target_position.qz
       << "},"
       << "\"current_test\":\"" << escape_json(g_robot_status.current_test) << "\","
       << "\"last_error\":\"" << escape_json(g_robot_status.last_error) << "\","
       << "\"strategy\":\"" << (g_strategy.current_mode == GameStrategy::DEFENSIVE ? "defensive" :
                                g_strategy.current_mode == GameStrategy::OFFENSIVE ? "offensive" : "intercept") << "\""
       << "}";
    return os.str();
}

std::string detection_status_json() {
    std::lock_guard<std::mutex> lock(g_shared.mutex);

    cv::Point2f table_xy = g_table_calib.ready
                          ? g_table_calib.pixelToTableMeters(g_shared.puck_position)
                          : cv::Point2f((g_shared.puck_position.x - 320.0f)/1000.0f,
                                       (g_shared.puck_position.y - 240.0f)/1000.0f);

    std::ostringstream os;
    os << "{"
       << "\"puck_detected\":" << (g_shared.puck_detected ? "true" : "false") << ","
       << "\"pixel_position\":{\"x\":" << g_shared.puck_position.x << ",\"y\":" << g_shared.puck_position.y << "},"
       << "\"table_position_m\":{\"x\":" << table_xy.x << ",\"y\":" << table_xy.y << "},"
       << "\"confidence\":" << g_shared.puck_confidence << ","
       << "\"velocity_px_per_s\":{\"x\":" << g_shared.puck_velocity.x << ",\"y\":" << g_shared.puck_velocity.y << "},"
       << "\"table_ready\":" << (g_table_calib.ready ? "true":"false") << ","
       << "\"frame_count\":" << g_shared.frame_count
       << "}";
    return os.str();
}

std::string mat_to_jpeg(const cv::Mat& img, int quality = 80) {
    std::vector<uchar> jpeg_buffer;
    cv::imencode(".jpg", img, jpeg_buffer, {cv::IMWRITE_JPEG_QUALITY, quality});
    return std::string(jpeg_buffer.begin(), jpeg_buffer.end());
}

void send_mjpeg_stream(int client_socket) {
    std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Connection: keep-alive\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n";
    send(client_socket, header.c_str(), header.length(), 0);
    
    while (g_running) {
        cv::Mat frame, display_frame;
        cv::Point2f puck_pos;
        cv::Point2f puck_vel;
        bool detected;
        
        {
            std::lock_guard<std::mutex> lock(g_shared.mutex);
            if (g_shared.current_frame.empty()) {
                std::this_thread::sleep_for(milliseconds(33));
                continue;
            }
            frame = g_shared.current_frame.clone();
            puck_pos = g_shared.puck_position;
            puck_vel = g_shared.puck_velocity;
            detected = g_shared.puck_detected;
        }
        
        display_frame = frame.clone();
        
        // Draw detection overlay
        cv::rectangle(display_frame, g_shared.table_roi, cv::Scalar(0, 255, 0), 2);
        
        if (detected) {
            cv::circle(display_frame, puck_pos, 10, cv::Scalar(0, 0, 255), 2);
            cv::circle(display_frame, puck_pos, 3, cv::Scalar(0, 0, 255), -1);
            
            // Draw velocity vector
            cv::Point2f vel_end = puck_pos + puck_vel * 0.1f;
            cv::arrowedLine(display_frame, puck_pos, vel_end, cv::Scalar(0, 255, 255), 2);
        }
        
        // Draw robot status
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            std::string robot_info = std::string("Robot: ") + (g_robot_status.is_connected ? "CONNECTED" : "DISCONNECTED");
            if (g_robot_status.is_moving) robot_info += " | MOVING";
            robot_info += " | " + g_robot_status.current_test;
            cv::putText(display_frame, robot_info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }
        
        // Draw table calibration overlay
        if (g_table_calib.ready) {
            auto corners_mm = g_table_calib.table_corners_mm();
            std::vector<cv::Point> poly;
            for (auto& corner : corners_mm) {
                cv::Point2f px_pt(corner.x * 640.0f / g_table_calib.width_mm, 
                                 corner.y * 480.0f / g_table_calib.height_mm);
                poly.emplace_back((int)px_pt.x, (int)px_pt.y);
            }
            for (int i = 0; i < 4; ++i) {
                cv::line(display_frame, poly[i], poly[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            }
        }
        
        std::vector<uchar> jpeg_buffer;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", display_frame, jpeg_buffer, compression_params);
        
        std::string frame_header =
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: " + std::to_string(jpeg_buffer.size()) + "\r\n"
            "\r\n";
            
        if (send(client_socket, frame_header.c_str(), frame_header.length(), 0) <= 0) break;
        if (send(client_socket, jpeg_buffer.data(), jpeg_buffer.size(), 0) <= 0) break;
        if (send(client_socket, "\r\n", 2, 0) <= 0) break;
        
        std::this_thread::sleep_for(milliseconds(33)); // ~30 FPS
    }
}

// Parse simple numbers from JSON body
std::vector<double> parse_numbers(const std::string& body) {
    std::vector<double> nums; 
    std::string cur;
    auto flush = [&]{ 
        if(!cur.empty()){ 
            try{ nums.push_back(std::stod(cur)); }catch(...){} 
            cur.clear(); 
        } 
    };
    for(char c: body){
        if(std::isdigit((unsigned char)c) || c=='-'||c=='+'||c=='.'||c=='e'||c=='E')
            cur.push_back(c);
        else flush();
    }
    flush();
    return nums;
}

std::string get_html_page() {
    return R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Fast Puck Tracker & Robot Control</title>
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
        .position-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; align-items: center; margin: 15px 0; }
        .position-grid label { font-weight: bold; text-align: center; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 20px; }
        .slider-container { margin: 10px 0; }
        .slider-container label { display: inline-block; width: 150px; }
        .pill { display:inline-block; padding:2px 8px; border-radius:12px; background:#ddd; font-size:12px; margin-left:6px; }
        .pill.ok { background:#c8f7c5; }
        .pill.no { background:#ffd5cc; }
        table { width: 100%; border-collapse: collapse; margin: 10px 0; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }
        th { background-color: #f2f2f2; }
        .real-time { font-family: monospace; font-size: 14px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 Fast Puck Tracker & ABB Robot Control</h1>
        
        <div class="status-grid">
            <div class="section">
                <h2>📹 Video Stream <span id="tbl-ready" class="pill">table: ?</span></h2>
                <div class="video-container">
                    <img id="video-stream" src="/stream" alt="Camera Stream" style="max-width:100%; border:2px solid #333; border-radius:5px; cursor:crosshair;">
                </div>
                <div class="controls">
                    <button onclick="refreshStream()">🔄 Refresh</button>
                    <button onclick="showCalibrationImage('mask')">🎭 Show Mask</button>
                    <button onclick="showCalibrationImage('result')">🎯 Show Result</button>
                </div>
            </div>

            <div class="section">
                <h2>🎯 Detection Status</h2>
                <div id="detection-status" class="info">
                    <p><strong>Status:</strong> <span id="d-detected">–</span></p>
                    <p><strong>Position (px):</strong> <span id="d-position-px">–</span></p>
                    <p><strong>Position (m, table):</strong> <span id="d-position-m">–</span></p>
                    <p><strong>Velocity (px/s):</strong> <span id="d-velocity">–</span></p>
                    <p><strong>Confidence:</strong> <span id="d-confidence">–</span></p>
                    <p><strong>Frame Count:</strong> <span id="d-frames">–</span></p>
                </div>
            </div>
            
            <div class="section">
                <h2>🤖 Robot Status</h2>
                <div id="robot-status" class="info">
                    <p><strong>Connection:</strong> <span id="r-connected">–</span></p>
                    <p><strong>IP:Port:</strong> <span id="r-ip">–</span>:<span id="r-port">–</span></p>
                    <p><strong>Moving:</strong> <span id="r-moving">–</span></p>
                    <p><strong>Current Task:</strong> <span id="r-task">–</span></p>
                    <p><strong>Strategy:</strong> <span id="r-strategy">–</span></p>
                    <p><strong>Last Error:</strong> <span id="r-error">–</span></p>
                </div>
            </div>
        </div>

        <div class="controls-grid">
            <div class="section">
                <h2>⚙️ Detection Calibration</h2>
                <div class="controls">
                    <select id="detection-method">
                        <option value="color">Color Detection (HSV)</option>
                        <option value="background">Background Subtraction</option>
                    </select>
                    <button onclick="updateDetectionMethod()">Update Method</button>
                </div>

                <div id="hsv-controls">
                    <h3>HSV Parameters</h3>
                    <div class="slider-container">
                        <label>H Min:</label><input type="range" id="h-min" min="0" max="179" value="0" oninput="updateHSV()">
                        <span id="h-min-val">0</span>
                    </div>
                    <div class="slider-container">
                        <label>S Min:</label><input type="range" id="s-min" min="0" max="255" value="100" oninput="updateHSV()">
                        <span id="s-min-val">100</span>
                    </div>
                    <div class="slider-container">
                        <label>V Min:</label><input type="range" id="v-min" min="0" max="255" value="100" oninput="updateHSV()">
                        <span id="v-min-val">100</span>
                    </div>
                    <div class="slider-container">
                        <label>H Max:</label><input type="range" id="h-max" min="0" max="179" value="10" oninput="updateHSV()">
                        <span id="h-max-val">10</span>
                    </div>
                    <div class="slider-container">
                        <label>S Max:</label><input type="range" id="s-max" min="0" max="255" value="255" oninput="updateHSV()">
                        <span id="s-max-val">255</span>
                    </div>
                    <div class="slider-container">
                        <label>V Max:</label><input type="range" id="v-max" min="0" max="255" value="255" oninput="updateHSV()">
                        <span id="v-max-val">255</span>
                    </div>
                    <div class="slider-container">
                        <label>Min Area:</label><input type="range" id="min-area" min="50" max="1000" value="100" oninput="updateArea()">
                        <span id="min-area-val">100</span>
                    </div>
                    <div class="slider-container">
                        <label>Max Area:</label><input type="range" id="max-area" min="1000" max="10000" value="5000" oninput="updateArea()">
                        <span id="max-area-val">5000</span>
                    </div>
                </div>
            </div]
            
            <div class="section">
                <h2>🤖 Robot Control</h2>
                
                <h3>Connection</h3>
                <div class="controls">
                    <input id="robot-ip" type="text" value="10.25.74.172" placeholder="Robot IP">
                    <input id="robot-port" type="number" value="5000" placeholder="Port">
                    <button onclick="connectRobot()">🔌 Connect</button>
                    <button onclick="disconnectRobot()" class="danger">❌ Disconnect</button>
                </div>
                
                <h3>Game Strategy</h3>
                <div class="controls">
                    <select id="game-strategy">
                        <option value="defensive">🛡️ Defensive</option>
                        <option value="offensive">⚔️ Offensive</option>
                        <option value="intercept">🎯 Intercept</option>
                    </select>
                    <button onclick="updateStrategy()">Update Strategy</button>
                    <button onclick="emergencyStop()" class="danger">🛑 EMERGENCY STOP</button>
                </div>

                <h3>Manual Control</h3>
                <div class="position-grid">
                    <label>X (m):</label>
                    <input id="manual-x" type="number" class="position-input" step="0.01" value="0" min="-0.6" max="0.6">
                    <span id="manual-x-display">0.00m</span>
                    <label>Y (m):</label>
                    <input id="manual-y" type="number" class="position-input" step="0.01" value="0" min="-0.4" max="0.4">
                    <span id="manual-y-display">0.00m</span>
                    <label>Z (m):</label>
                    <input id="manual-z" type="number" class="position-input" step="0.01" value="0.05" min="0.02" max="0.1">
                    <span id="manual-z-display">0.05m</span>
                </div>
                <div class="controls">
                    <label>Speed: </label>
                    <input id="manual-speed" type="range" min="0.1" max="1.0" step="0.1" value="0.5">
                    <span id="speed-value">0.5</span>
                </div>
                <div class="controls">
                    <button onclick="moveToPosition()">🎯 Move To Position</button>
                    <button onclick="moveHome()">🏠 Move Home</button>
                    <button onclick="executeStrike()">⚡ Strike Now!</button>
                </div>
            </div>
        </div>

        <div class="section">
            <h2>📊 Table Calibration (4 corners)</h2>
            <p>Click 4 table corners in the video above, then set dimensions and origin.</p>
            <div class="controls">
                Width (mm): <input id="tbl-w" type="number" value="1000" style="width:100px">
                Height (mm): <input id="tbl-h" type="number" value="500" style="width:100px">
                Origin:
                <select id="tbl-origin">
                  <option value="0">Corner #1</option>
                  <option value="1">Corner #2</option>
                  <option value="2">Corner #3</option>
                  <option value="3">Corner #4</option>
                </select>
                Direction:
                <select id="tbl-cw">
                  <option value="1">Clockwise</option>
                  <option value="0">Counter-clockwise</option>
                </select>
            </div>
            <div class="controls">
                <button onclick="startCornerPick()">🎯 Pick Corners</button>
                <button onclick="resetCorners()">♻️ Reset</button>
                <button onclick="sendTableCalibration()">✅ Save Calibration</button>
            </div>
            <div class="info">
                Collected corners: <span id="corners-count">0</span> / 4
            </div>
        </div>

        <div class="section">
            <h2>📈 Current Robot Position</h2>
            <div class="real-time">
                <table>
                    <tr><th>X</th><th>Y</th><th>Z</th><th>Q0</th><th>QX</th><th>QY</th><th>QZ</th></tr>
                    <tr>
                        <td id="pos-x">0.000</td>
                        <td id="pos-y">0.000</td>
                        <td id="pos-z">0.000</td>
                        <td id="pos-q0">1.000</td>
                        <td id="pos-qx">0.000</td>
                        <td id="pos-qy">0.000</td>
                        <td id="pos-qz">0.000</td>
                    </tr>
                </table>
            </div>
        </div>
    </div>

    <script>
        // Status update functions
        async function fetchDetectionStatus() {
            try {
                const r = await fetch('/detection/status');
                const j = await r.json();
                document.getElementById('d-detected').textContent = j.puck_detected ? 'Detected' : 'Not detected';
                document.getElementById('d-position-px').textContent = `(${j.pixel_position.x.toFixed(1)}, ${j.pixel_position.y.toFixed(1)})`;
                document.getElementById('d-position-m').textContent = `(${j.table_position_m.x.toFixed(3)}, ${j.table_position_m.y.toFixed(3)})`;
                document.getElementById('d-velocity').textContent = `(${j.velocity_px_per_s.x.toFixed(1)}, ${j.velocity_px_per_s.y.toFixed(1)})`;
                document.getElementById('d-confidence').textContent = `${(j.confidence * 100).toFixed(1)}%`;
                document.getElementById('d-frames').textContent = j.frame_count;
                
                const pill = document.getElementById('tbl-ready');
                pill.textContent = j.table_ready ? 'table: ready' : 'table: not ready';
                pill.className = 'pill ' + (j.table_ready ? 'ok' : 'no');
            } catch (e) {
                document.getElementById('d-detected').textContent = 'Error';
            }
        }

        async function fetchRobotStatus() {
            try {
                const r = await fetch('/robot/status');
                const j = await r.json();
                
                document.getElementById('r-connected').textContent = j.connected ? 'Connected' : 'Disconnected';
                document.getElementById('r-ip').textContent = j.ip;
                document.getElementById('r-port').textContent = j.port;
                document.getElementById('r-moving').textContent = j.moving ? 'Yes' : 'No';
                document.getElementById('r-task').textContent = j.current_test || 'None';
                document.getElementById('r-strategy').textContent = j.strategy || 'None';
                document.getElementById('r-error').textContent = j.last_error || '–';
                
                // Update position display
                if (j.current_position) {
                    document.getElementById('pos-x').textContent = j.current_position.x.toFixed(3);
                    document.getElementById('pos-y').textContent = j.current_position.y.toFixed(3);
                    document.getElementById('pos-z').textContent = j.current_position.z.toFixed(3);
                    document.getElementById('pos-q0').textContent = j.current_position.q0.toFixed(3);
                    document.getElementById('pos-qx').textContent = j.current_position.qx.toFixed(3);
                    document.getElementById('pos-qy').textContent = j.current_position.qy.toFixed(3);
                    document.getElementById('pos-qz').textContent = j.current_position.qz.toFixed(3);
                }
            } catch (e) {
                document.getElementById('r-connected').textContent = 'Error';
                document.getElementById('r-error').textContent = e.toString();
            }
        }

        // Calibration functions
        function updateHSV() {
            const hMin = document.getElementById('h-min').value;
            const sMin = document.getElementById('s-min').value;
            const vMin = document.getElementById('v-min').value;
            const hMax = document.getElementById('h-max').value;
            const sMax = document.getElementById('s-max').value;
            const vMax = document.getElementById('v-max').value;

            updateSliderValues();

            fetch('/calibration/hsv', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    hsv_lower: [parseInt(hMin), parseInt(sMin), parseInt(vMin)],
                    hsv_upper: [parseInt(hMax), parseInt(sMax), parseInt(vMax)]
                })
            });
        }

        function updateArea() {
            const minArea = document.getElementById('min-area').value;
            const maxArea = document.getElementById('max-area').value;

            document.getElementById('min-area-val').textContent = minArea;
            document.getElementById('max-area-val').textContent = maxArea;

            fetch('/calibration/area', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    min_area: parseInt(minArea),
                    max_area: parseInt(maxArea)
                })
            });
        }

        function updateSliderValues() {
            document.getElementById('h-min-val').textContent = document.getElementById('h-min').value;
            document.getElementById('s-min-val').textContent = document.getElementById('s-min').value;
            document.getElementById('v-min-val').textContent = document.getElementById('v-min').value;
            document.getElementById('h-max-val').textContent = document.getElementById('h-max').value;
            document.getElementById('s-max-val').textContent = document.getElementById('s-max').value;
            document.getElementById('v-max-val').textContent = document.getElementById('v-max').value;
        }

        function updateDetectionMethod() {
            const method = document.getElementById('detection-method').value;
            fetch('/calibration/method', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({method: method})
            });

            const hsvControls = document.getElementById('hsv-controls');
            hsvControls.style.display = method === 'color' ? 'block' : 'none';
        }

        function showCalibrationImage(type) {
            window.open('/calibration/' + type, '_blank');
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
                
                if (result.connected) {
                    alert('Robot connected successfully!');
                } else {
                    alert('Failed to connect: ' + (result.last_error || 'Unknown error'));
                }
                
            } catch (error) {
                alert('Connection failed: ' + error.message);
            }
        }

        async function disconnectRobot() {
            try {
                await fetch('/robot/disconnect', { method: 'POST' });
            } catch (error) {
                alert('Disconnect failed: ' + error.message);
            }
        }

        async function updateStrategy() {
            const strategy = document.getElementById('game-strategy').value;
            try {
                await fetch('/robot/strategy', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({strategy: strategy})
                });
            } catch (error) {
                alert('Strategy update failed: ' + error.message);
            }
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

        async function moveToPosition() {
            const position = {
                x: parseFloat(document.getElementById('manual-x').value),
                y: parseFloat(document.getElementById('manual-y').value),
                z: parseFloat(document.getElementById('manual-z').value),
                speed: parseFloat(document.getElementById('manual-speed').value)
            };
            
            try {
                const response = await fetch('/robot/manual_move', {
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
            document.getElementById('manual-z').value = '0.05';
            updateManualDisplays();
            await moveToPosition();
        }

        async function executeStrike() {
            try {
                await fetch('/robot/strike', { method: 'POST' });
            } catch (error) {
                alert('Strike failed: ' + error.message);
            }
        }

        // Table calibration
        let cornerPickEnabled = false;
        let pickedCorners = [];

        function startCornerPick() {
            pickedCorners = [];
            document.getElementById('corners-count').textContent = '0';
            cornerPickEnabled = true;
            alert('Click 4 table corners in the video stream.');
        }

        function resetCorners() {
            pickedCorners = [];
            document.getElementById('corners-count').textContent = '0';
        }

        function imageClickToPixel(img, evt) {
            const rect = img.getBoundingClientRect();
            const scaleX = img.naturalWidth  / rect.width;
            const scaleY = img.naturalHeight / rect.height;
            const x = (evt.clientX - rect.left) * scaleX;
            const y = (evt.clientY - rect.top)  * scaleY;
            return {x:x, y:y};
        }

        document.getElementById('video-stream').addEventListener('click', function(evt){
            if (!cornerPickEnabled) return;
            const p = imageClickToPixel(this, evt);
            pickedCorners.push(p);
            document.getElementById('corners-count').textContent = pickedCorners.length.toString();
            if (pickedCorners.length === 4) {
                cornerPickEnabled = false;
                alert('Collected 4 points. Now click "Save Calibration".');
            }
        });

        async function sendTableCalibration() {
            if (pickedCorners.length !== 4) { alert('First click 4 corners.'); return; }
            const w  = parseFloat(document.getElementById('tbl-w').value);
            const h  = parseFloat(document.getElementById('tbl-h').value);
            const oi = parseInt(document.getElementById('tbl-origin').value);
            const cw = parseInt(document.getElementById('tbl-cw').value);

            const body = {
              points: pickedCorners,
              width_mm: w,
              height_mm: h,
              origin_index: oi,
              clockwise: cw
            };

            const r = await fetch('/table/calibrate', {
              method:'POST',
              headers: {'Content-Type':'application/json'},
              body: JSON.stringify(body)
            });
            const j = await r.json();
            if (j.ready) alert('Table calibration successful!');
            else alert('Table calibration failed.');
        }

        // Manual position display updates
        function updateManualDisplays() {
            document.getElementById('manual-x-display').textContent = parseFloat(document.getElementById('manual-x').value).toFixed(2) + 'm';
            document.getElementById('manual-y-display').textContent = parseFloat(document.getElementById('manual-y').value).toFixed(2) + 'm';
            document.getElementById('manual-z-display').textContent = parseFloat(document.getElementById('manual-z').value).toFixed(2) + 'm';
            document.getElementById('speed-value').textContent = document.getElementById('manual-speed').value;
        }

        document.getElementById('manual-x').addEventListener('input', updateManualDisplays);
        document.getElementById('manual-y').addEventListener('input', updateManualDisplays);
        document.getElementById('manual-z').addEventListener('input', updateManualDisplays);
        document.getElementById('manual-speed').addEventListener('input', updateManualDisplays);

        function refreshStream() {
            var img = document.getElementById('video-stream');
            img.src = '/stream?t=' + new Date().getTime();
        }

        // Auto-refresh status
        setInterval(fetchDetectionStatus, 1000);
        setInterval(fetchRobotStatus, 2000);
        
        // Initialize
        window.addEventListener('load', function() {
            fetchDetectionStatus();
            fetchRobotStatus();
            updateManualDisplays();
            updateSliderValues();
        });
    </script>
</body>
</html>
)html";
}

void handle_client(int client_socket) {
    char buffer[8192] = {0};
    recv(client_socket, buffer, sizeof(buffer)-1, 0);
    std::string request(buffer);

    if (request.find("GET / ") == 0) {
        send_http_response(client_socket, "text/html", get_html_page());
    }
    else if (request.find("GET /stream") == 0) {
        send_mjpeg_stream(client_socket);
        return; // Don't close socket immediately for stream
    }
    else if (request.find("GET /detection/status") == 0) {
        send_http_response(client_socket, "application/json", detection_status_json());
    }
    else if (request.find("GET /robot/status") == 0) {
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.find("GET /calibration/mask") == 0) {
        std::lock_guard<std::mutex> lock(g_shared.mutex);
        if (!g_shared.detection_mask.empty()) {
            send_http_response(client_socket, "image/jpeg", mat_to_jpeg(g_shared.detection_mask));
        } else {
            send_http_response(client_socket, "text/plain", "no mask available");
        }
    }
    else if (request.find("GET /calibration/result") == 0) {
        std::lock_guard<std::mutex> lock(g_shared.mutex);
        if (!g_shared.current_frame.empty() && !g_shared.detection_mask.empty()) {
            cv::Mat result, colorMask;
            cv::cvtColor(g_shared.detection_mask, colorMask, cv::COLOR_GRAY2BGR);
            cv::addWeighted(g_shared.current_frame, 1.0, colorMask, 0.5, 0, result);
            send_http_response(client_socket, "image/jpeg", mat_to_jpeg(result));
        } else {
            send_http_response(client_socket, "text/plain", "no result available");
        }
    }
    else if (request.find("POST /calibration/hsv") == 0) {
        // Parse HSV values from request body (simplified)
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            auto nums = parse_numbers(body);
            if (nums.size() >= 6) {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                g_shared.hsv_lower = cv::Scalar(nums[0], nums[1], nums[2]);
                g_shared.hsv_upper = cv::Scalar(nums[3], nums[4], nums[5]);
            }
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/area") == 0) {
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            auto nums = parse_numbers(body);
            if (nums.size() >= 2) {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                g_shared.min_area = (int)nums[0];
                g_shared.max_area = (int)nums[1];
            }
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/method") == 0) {
        std::lock_guard<std::mutex> lock(g_shared.mutex);
        g_shared.detection_method = (g_shared.detection_method == DetectionMethod::COLOR_THRESHOLD) ?
            DetectionMethod::BACKGROUND_SUB : DetectionMethod::COLOR_THRESHOLD;
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    // ===== Table Calibration =====
    else if (request.find("POST /table/calibrate") == 0) {
        auto p = request.find("\r\n\r\n");
        if (p == std::string::npos) {
            send_http_response(client_socket, "application/json", "{\"status\":\"error\",\"msg\":\"no body\"}");
        } else {
            std::string body = request.substr(p+4);
            auto nums = parse_numbers(body);
            // Expect: x1 y1 x2 y2 x3 y3 x4 y4 width height origin clockwise
            if (nums.size() < 12) {
                send_http_response(client_socket, "application/json", "{\"status\":\"error\",\"msg\":\"need 12 numbers\"}");
            } else {
                g_table_calib.img_pts.clear();
                g_table_calib.img_pts.emplace_back((float)nums[0], (float)nums[1]);
                g_table_calib.img_pts.emplace_back((float)nums[2], (float)nums[3]);
                g_table_calib.img_pts.emplace_back((float)nums[4], (float)nums[5]);
                g_table_calib.img_pts.emplace_back((float)nums[6], (float)nums[7]);
                g_table_calib.width_mm     = (float)nums[8];
                g_table_calib.height_mm    = (float)nums[9];
                g_table_calib.origin_index = std::clamp<int>((int)std::lround(nums[10]), 0, 3);
                g_table_calib.clockwise    = (nums[11] >= 0.5);
                g_table_calib.computeHomography();

                send_http_response(client_socket, "application/json",
                    std::string("{\"status\":\"ok\",\"ready\":") + (g_table_calib.ready?"true":"false") + "}");
            }
        }
    }
    // ===== Robot Control Endpoints =====
    else if (request.rfind("POST /robot/connect", 0) == 0) {
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

        bool ok = false;
        std::string err;

    #ifdef USE_ABB
        {
            std::lock_guard<std::mutex> lk(g_robot_mutex);
            if (g_robot) { 
                try { 
                    g_robot->disconnect(); 
                } catch(...) {} 
                g_robot = nullptr; 
            }
            g_connection.ip = ip;
            g_connection.port = port;
            g_robot_status.last_error.clear();
        }
        
        try {
            std::cout << "[ROBOT] Connecting to " << ip << ":" << port << "\n";
            
            // Create ABB controller
            auto* r = new AbbController(ip, port);
            
            if (r->connect()) {
                std::cout << "[ROBOT] Connected successfully\n";
                
                // Set initial speed
                r->setSpeed(100.0, 50.0);
                
                // Query current position if possible
                std::array<double, 7> pose;
                if (r->getCartesian(pose)) {
                    std::cout << "[ROBOT] Initial position: (" 
                              << pose[0] << ", " << pose[1] << ", " << pose[2] << ") m\n";
                    
                    // Update robot status (store in mm for UI table)
                    g_robot_status.current_position.x = pose[0] * 1000.0f;  // m to mm
                    g_robot_status.current_position.y = pose[1] * 1000.0f;
                    g_robot_status.current_position.z = pose[2] * 1000.0f;
                    g_robot_status.current_position.q0 = pose[3];
                    g_robot_status.current_position.qx = pose[4];
                    g_robot_status.current_position.qy = pose[5];
                    g_robot_status.current_position.qz = pose[6];
                    
                    // Update strategy robot position
                    g_strategy.robot_position = cv::Point2f(pose[0], pose[1]);
                }
                
                std::lock_guard<std::mutex> lk(g_robot_mutex);
                g_robot = r; 
                g_robot_status.is_connected = true;
                ok = true;
            } else {
                delete r; 
                err = "connect() returned false";
            }
        } catch (const std::exception& e) { 
            err = e.what(); 
        }

        if (!ok) {
            std::lock_guard<std::mutex> lk(g_robot_mutex);
            g_robot_status.last_error = err.empty() ? "Unknown error" : err;
            g_robot_status.is_connected = false;
            std::cout << "[ROBOT] Connection failed: " << g_robot_status.last_error << "\n";
        }
    #else
        {
            std::lock_guard<std::mutex> lk(g_robot_mutex);
            g_robot_status.last_error = "ABB disabled at compile time (USE_ABB=OFF)";
            g_robot_status.is_connected = false;
        }
    #endif
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.find("POST /robot/disconnect") == 0) {
        {
            std::lock_guard<std::mutex> lock(g_robot_mutex);
            if (g_robot) { 
                try { g_robot->disconnect(); } catch(...) {} 
                g_robot = nullptr; 
            }
            g_robot_status.is_connected = false;
            g_robot_status.last_error = "Disconnected by user";
        }
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.find("POST /robot/strategy") == 0) {
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            if (body.find("defensive") != std::string::npos)      g_strategy.current_mode = GameStrategy::DEFENSIVE;
            else if (body.find("offensive") != std::string::npos) g_strategy.current_mode = GameStrategy::OFFENSIVE;
            else if (body.find("intercept") != std::string::npos) g_strategy.current_mode = GameStrategy::INTERCEPT;
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"strategy_updated\"}");
    }
    else if (request.find("POST /robot/emergency_stop") == 0) {
        std::lock_guard<std::mutex> lock(g_robot_mutex);
        if (g_robot) {
            try {
                g_robot->disconnect();
                g_robot = nullptr;
                g_robot_status.last_error = "Emergency stop executed (connection closed)";
            } catch (const std::exception& e) {
                g_robot_status.last_error = std::string("Emergency stop failed: ") + e.what();
            }
        } else {
            g_robot_status.last_error = "Emergency stop (no active connection)";
        }
        g_robot_status.is_connected = false;
        g_robot_status.is_moving = false;
        send_http_response(client_socket, "application/json", "{\"status\":\"emergency_stop\"}");
    }
    else if (request.find("POST /robot/manual_move") == 0) {
        // Parse x,y,z,speed from body (meters)
        float x=0.0f, y=0.0f, z=0.05f, speed=0.5f;
        auto p = request.find("\r\n\r\n");
        if (p != std::string::npos) {
            std::string body = request.substr(p+4);
            auto nums = parse_numbers(body);
            if (nums.size() >= 1) x = (float)nums[0];
            if (nums.size() >= 2) y = (float)nums[1];
            if (nums.size() >= 3) z = (float)nums[2];
            if (nums.size() >= 4) speed = (float)nums[3];
        }
        RobotCommand cmd;
        cmd.x = x; cmd.y = y; cmd.z = z; cmd.speed = speed;
        cmd.name = "Manual Move";
        sendRobotCommand(cmd);
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /robot/strike") == 0) {
        cv::Point2f puck_pos, velocity;
        {
            std::lock_guard<std::mutex> lock(g_shared.mutex);
            puck_pos = g_shared.puck_position;
            velocity = g_shared.puck_velocity;
        }
        RobotCommand cmd = calculateRobotCommand(puck_pos, velocity);
        cmd.execute_strike = true; 
        cmd.speed = 1.0f;
        cmd.name = "Strike Command";
        sendRobotCommand(cmd);
        send_http_response(client_socket, "application/json", "{\"status\":\"strike_executed\"}");
    }
    else {
        send_http_response(client_socket, "text/html", "<h1>404 Not Found</h1>");
    }

    close(client_socket);
}

void http_server_thread(int port) {
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        std::cerr << "Failed to create HTTP server socket!" << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "HTTP server bind failed!" << std::endl;
        close(server_socket);
        return;
    }
    
    if (listen(server_socket, 5) < 0) {
        std::cerr << "HTTP server listen failed!" << std::endl;
        close(server_socket);
        return;
    }

    std::cout << "HTTP interface with robot control running on port " << port << std::endl;

    while (g_running) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
        if (client_socket < 0) continue;
        
        std::thread client_thread(handle_client, client_socket);
        client_thread.detach();
    }

    close(server_socket);
}

int main(int argc, char* argv[]) {
    Config cfg = parse_args(argc, argv);
    
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // Load YAML config
    YAML::Node yaml_config;
    try {
        yaml_config = YAML::LoadFile(cfg.config_file);
    } catch (const std::exception& e) {
        std::cerr << "Warning: Could not load config file: " << e.what() << std::endl;
    }
    
    // Initialize shared data with config values
    if (yaml_config["detection"]["hsv_lower"]) {
        auto lower = yaml_config["detection"]["hsv_lower"].as<std::vector<int>>();
        g_shared.hsv_lower = cv::Scalar(lower[0], lower[1], lower[2]);
    }
    if (yaml_config["detection"]["hsv_upper"]) {
        auto upper = yaml_config["detection"]["hsv_upper"].as<std::vector<int>>();
        g_shared.hsv_upper = cv::Scalar(upper[0], upper[1], upper[2]);
    }
    g_shared.min_area = yaml_config["detection"]["min_area"].as<int>(100);
    g_shared.max_area = yaml_config["detection"]["max_area"].as<int>(5000);
    g_shared.min_circularity = yaml_config["detection"]["min_circularity"].as<float>(0.7f);
    
    if (yaml_config["detection"]["roi"]) {
        auto roi = yaml_config["detection"]["roi"].as<std::vector<int>>();
        g_shared.table_roi = cv::Rect(roi[0], roi[1], roi[2], roi[3]);
    }
    
    g_shared.detection_method = (cfg.method == "color") ? 
        DetectionMethod::COLOR_THRESHOLD : DetectionMethod::BACKGROUND_SUB;
    
    // Initialize table calibration from config
    if (yaml_config["table"]["width_mm"]) {
        g_table_calib.width_mm = yaml_config["table"]["width_mm"].as<float>();
    }
    if (yaml_config["table"]["height_mm"]) {
        g_table_calib.height_mm = yaml_config["table"]["height_mm"].as<float>();
    }
    setupDefaultTableCalibration();

    // Open output file
    std::ofstream output_file;
    if (cfg.save_output) {
        output_file.open(cfg.output_file);
        output_file << "timestamp,x,y,vx,vy,detected" << std::endl;
    }
    
    // Initialize components
    auto capture = std::make_unique<FrameCapture>(cfg.backend, cfg.width, 
                                                  cfg.height, cfg.fps);
    
    if (!capture->initialize()) {
        std::cerr << "Failed to initialize capture device!" << std::endl;
        return -1;
    }
    
    auto detector = std::make_unique<Detector>(yaml_config);
    auto tracker = std::make_unique<KalmanTracker>(yaml_config);
    auto stats = std::make_shared<PerfStats>();
    
    // Create queue
    LockFreeQueue<FrameData> capture_queue(cfg.queue_size);
    
    std::cout << "Starting fast puck tracker with ABB robot control..." << std::endl;
    std::cout << "Resolution: " << cfg.width << "x" << cfg.height 
              << " @ " << cfg.fps << " FPS" << std::endl;
    std::cout << "Detection: " << cfg.method << std::endl;
#ifdef USE_ABB
    std::cout << "ABB Robot Control: ENABLED" << std::endl;
#else
    std::cout << "ABB Robot Control: SIMULATION MODE (USE_ABB=OFF)" << std::endl;
#endif
    std::cout << "HTTP interface: http://localhost:" << cfg.http_port << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    // Start HTTP server thread
    std::thread http_thread(http_server_thread, cfg.http_port);
    
    // Start processing threads
    std::thread capture_t(capture_thread, capture.get(), &capture_queue, 
                         std::ref(g_running), stats.get());

    // Atomic counter for processed frames
    std::atomic<int> frames_processed{0};
    
    std::thread process_t(process_thread, &capture_queue,
                         detector.get(), tracker.get(), std::ref(g_running),
                         stats.get(), std::ref(frames_processed),
                         cfg.save_output ? &output_file : nullptr);
    
    // Performance monitoring loop
    auto last_stats_update = high_resolution_clock::now();
    
    while (g_running) {
        std::this_thread::sleep_for(seconds(1));
        
        auto now = high_resolution_clock::now();
        auto elapsed = duration_cast<seconds>(now - last_stats_update).count();
        
        if (elapsed >= 5 && cfg.enable_profiling) { // Every 5 seconds
            int processed = frames_processed.exchange(0);
            float fps = processed / static_cast<float>(elapsed);
            
            // Update shared stats
            {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                g_shared.fps = fps;
                g_shared.latency_ms = stats->getLatency();
                g_shared.dropped_frames = stats->dropped_frames;
            }
            
            std::cout << "\n[STATS] FPS: " << std::fixed << std::setprecision(1) << fps
                     << " | Latency: " << stats->getLatency() << "ms"
                     << " | Drops: " << stats->dropped_frames
                     << " | Cap: " << stats->capture_time << "μs"
                     << " | Proc: " << stats->process_time << "μs" << std::endl;
            
            last_stats_update = now;
        }
    }
    
    // Cleanup
    http_thread.join();
    capture_t.join();
    process_t.join();
    
    if (output_file.is_open()) {
        output_file.close();
    }
    
    {
        std::lock_guard<std::mutex> lock(g_robot_mutex);
        if (g_robot) {
            try { g_robot->disconnect(); } catch(...) {}
            g_robot = nullptr;
        }
    }
    
    std::cout << "\nShutdown complete." << std::endl;
    return 0;
}
