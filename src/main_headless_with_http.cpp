//main_headless_with_http.cpp
// Szybki headless tracker z HTTP interfejsem kalibracji
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

#include "capture.hpp"
#include "detector.hpp"
#include "tracker.hpp"
#include "utils.hpp"
#include "lock_free_queue.hpp"

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

// Shared data between detection and HTTP threads
struct SharedData {
    std::mutex mutex;
    
    // Current detection settings (can be modified via HTTP)
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
                   std::atomic<int>& frames_processed,                 // <-- added
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
            {
                std::lock_guard<std::mutex> lock(g_shared.mutex);
                g_shared.puck_position = tracker->getPosition();
                g_shared.puck_velocity = tracker->getVelocity();
                g_shared.puck_detected = detected;
                g_shared.puck_confidence = detected ? 1.0f : 0.0f;
                
                if (!mask.empty()) {
                    g_shared.detection_mask = cv::Mat::zeros(frame_data.frame.size(), CV_8UC1);
                    mask.copyTo(g_shared.detection_mask(roi));
                }
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

            frames_processed++;  // <-- added
            
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

// HTTP Helper Functions
void send_http_response(int client_socket, const std::string& content_type, const std::string& content) {
    std::string response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + std::to_string(content.length()) + "\r\n"
        "Connection: close\r\n"
        "\r\n" + content;
    send(client_socket, response.c_str(), response.length(), 0);
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
        bool detected;
        
        {
            std::lock_guard<std::mutex> lock(g_shared.mutex);
            if (g_shared.current_frame.empty()) {
                std::this_thread::sleep_for(milliseconds(33));
                continue;
            }
            frame = g_shared.current_frame.clone();
            puck_pos = g_shared.puck_position;
            detected = g_shared.puck_detected;
        }
        
        display_frame = frame.clone();
        
        // Draw detection overlay
        cv::rectangle(display_frame, g_shared.table_roi, cv::Scalar(0, 255, 0), 2);
        if (detected) {
            cv::circle(display_frame, puck_pos, 10, cv::Scalar(0, 0, 255), 2);
            cv::circle(display_frame, puck_pos, 3, cv::Scalar(0, 0, 255), -1);
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

std::string get_html_page() {
    return R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Puck Tracker - Calibration Interface</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin:0; padding:20px; background:#f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; background:#fff; padding:20px; border-radius:10px; }
        h1 { color:#333; text-align:center; }
        .video-container { text-align:center; margin:20px 0; }
        img { max-width:100%; border:2px solid #333; border-radius:5px; }
        .controls { text-align:center; margin:20px 0; }
        button { background:#4CAF50; border:none; color:#fff; padding:10px 20px; font-size:16px; margin:4px 2px; cursor:pointer; border-radius:5px; }
        button:hover { background:#45a049; }
        .info { background:#e7f3ff; border-left:6px solid #2196F3; margin: 20px 0; padding:10px; }
        input, select { border:1px solid #ccc; border-radius:6px; padding:8px; margin:2px; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 8px; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .slider-container { margin: 10px 0; }
        .slider-container label { display: inline-block; width: 150px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Puck Tracker - Fast Headless Mode</h1>

        <div class="status-grid">
            <div class="section">
                <h2>Video Stream</h2>
                <div class="video-container">
                    <img id="video-stream" src="/stream" alt="Camera Stream">
                </div>
                <div class="controls">
                    <button onclick="refreshStream()">Refresh Stream</button>
                    <button onclick="showCalibrationImage('mask')">Show Detection Mask</button>
                    <button onclick="showCalibrationImage('result')">Show Result</button>
                </div>
            </div>

            <div class="section">
                <h2>Detection Status</h2>
                <div id="detection-status" class="info">
                    <p><strong>Status:</strong> <span id="d-detected">-</span></p>
                    <p><strong>Position (px):</strong> <span id="d-position">-</span></p>
                    <p><strong>Velocity (px/s):</strong> <span id="d-velocity">-</span></p>
                    <p><strong>Frame Count:</strong> <span id="d-frames">-</span></p>
                </div>
            </div>
        </div>

        <div class="section">
            <h2>Detection Calibration</h2>
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
        </div>
    </div>

    <script>
        async function fetchDetectionStatus() {
            try {
                const r = await fetch('/status');
                const j = await r.json();
                document.getElementById('d-detected').textContent = j.detected ? 'Detected' : 'Not detected';
                document.getElementById('d-position').textContent = `(${j.position.x.toFixed(1)}, ${j.position.y.toFixed(1)})`;
                document.getElementById('d-velocity').textContent = `(${j.velocity.x.toFixed(1)}, ${j.velocity.y.toFixed(1)})`;
                document.getElementById('d-frames').textContent = j.frame_count;
            } catch (e) {
                document.getElementById('d-detected').textContent = 'Error';
            }
        }

        function updateHSV() {
            const hMin = document.getElementById('h-min').value;
            const sMin = document.getElementById('s-min').value;
            const vMin = document.getElementById('v-min').value;
            const hMax = document.getElementById('h-max').value;
            const sMax = document.getElementById('s-max').value;
            const vMax = document.getElementById('v-max').value;

            document.getElementById('h-min-val').textContent = hMin;
            document.getElementById('s-min-val').textContent = sMin;
            document.getElementById('v-min-val').textContent = vMin;
            document.getElementById('h-max-val').textContent = hMax;
            document.getElementById('s-max-val').textContent = sMax;
            document.getElementById('v-max-val').textContent = vMax;

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

        function updateDetectionMethod() {
            const method = document.getElementById('detection-method').value;
            fetch('/calibration/method', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({method: method})
            });
        }

        function showCalibrationImage(type) {
            window.open('/calibration/' + type, '_blank');
        }

        function refreshStream() {
            var img = document.getElementById('video-stream');
            img.src = '/stream?t=' + new Date().getTime();
        }

        // Auto-refresh status
        setInterval(fetchDetectionStatus, 1000);
        
        // Initialize
        window.addEventListener('load', fetchDetectionStatus);
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
    else if (request.find("GET /status") == 0) {
        std::lock_guard<std::mutex> lock(g_shared.mutex);
        std::ostringstream os;
        os << "{"
           << "\"detected\":" << (g_shared.puck_detected ? "true" : "false") << ","
           << "\"position\":{\"x\":" << g_shared.puck_position.x << ",\"y\":" << g_shared.puck_position.y << "},"
           << "\"velocity\":{\"x\":" << g_shared.puck_velocity.x << ",\"y\":" << g_shared.puck_velocity.y << "},"
           << "\"frame_count\":" << g_shared.frame_count
           << "}";
        send_http_response(client_socket, "application/json", os.str());
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
        // Parse HSV values from request body
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            // Simple parsing - in production you'd use proper JSON parser
            std::lock_guard<std::mutex> lock(g_shared.mutex);
            // For now, just acknowledge - you could implement proper JSON parsing here
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/area") == 0) {
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/method") == 0) {
        std::lock_guard<std::mutex> lock(g_shared.mutex);
        g_shared.detection_method = (g_shared.detection_method == DetectionMethod::COLOR_THRESHOLD) ?
            DetectionMethod::BACKGROUND_SUB : DetectionMethod::COLOR_THRESHOLD;
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
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

    std::cout << "HTTP calibration interface running on port " << port << std::endl;

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
    
    std::cout << "Starting fast puck tracker with HTTP interface..." << std::endl;
    std::cout << "Resolution: " << cfg.width << "x" << cfg.height 
              << " @ " << cfg.fps << " FPS" << std::endl;
    std::cout << "Detection: " << cfg.method << std::endl;
    std::cout << "HTTP interface: http://localhost:" << cfg.http_port << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    // Start HTTP server thread
    std::thread http_thread(http_server_thread, cfg.http_port);
    
    // Start processing threads
    std::thread capture_t(capture_thread, capture.get(), &capture_queue, 
                         std::ref(g_running), stats.get());

    // Atomic counter for processed frames (shared)
    std::atomic<int> frames_processed{0};                                 // <-- added
    
    std::thread process_t(process_thread, &capture_queue,
                         detector.get(), tracker.get(), std::ref(g_running),
                         stats.get(), std::ref(frames_processed),           // <-- added
                         cfg.save_output ? &output_file : nullptr);
    
    // Performance monitoring loop
    auto last_stats_update = high_resolution_clock::now();
    // removed: int frames_processed = 0;  // now using atomic
    
    while (g_running) {
        std::this_thread::sleep_for(seconds(1));
        
        auto now = high_resolution_clock::now();
        auto elapsed = duration_cast<seconds>(now - last_stats_update).count();
        
        if (elapsed >= 5 && cfg.enable_profiling) { // Every 5 seconds
            int processed = frames_processed.exchange(0);                   // <-- added
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
    
    std::cout << "\nShutdown complete." << std::endl;
    return 0;
}
