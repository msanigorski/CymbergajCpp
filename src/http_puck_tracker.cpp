#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <deque>
#include <cmath>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define closesocket close
#endif

struct PuckData {
    float x = -1, y = -1;
    float velocity_x = 0, velocity_y = 0;
    float speed = 0;
    bool detected = false;
    uint64_t timestamp = 0;
    
    PuckData() {
        auto now = std::chrono::steady_clock::now();
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    }
};

class FastPuckTracker {
private:
    int port;
    int server_socket;
    std::atomic<bool> running;
    std::thread server_thread;
    std::thread capture_thread;
    cv::VideoCapture cap;
    
    // Detection parameters
    cv::Scalar hsv_lower = cv::Scalar(0, 120, 120);
    cv::Scalar hsv_upper = cv::Scalar(10, 255, 255);
    int min_area = 15;
    int max_area = 500;
    double min_circularity = 0.6;
    
    // Thread-safe data
    std::mutex data_mutex;
    PuckData current_puck;
    std::deque<PuckData> trajectory;
    const size_t MAX_TRAJECTORY_POINTS = 30;
    
    // Performance counters
    std::atomic<double> processing_fps{0.0};
    std::atomic<int> frame_counter{0};
    std::chrono::steady_clock::time_point last_fps_update;
    PuckData previous_puck;
    
    // For serving images
    std::mutex frame_mutex;
    cv::Mat latest_frame;

    int extractJSONInt(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":";
        size_t pos = json.find(search);
        if (pos != std::string::npos) {
            pos += search.length();
            while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
            
            std::string number;
            if (pos < json.length() && json[pos] == '-') {
                number += json[pos++];
            }
            while (pos < json.length() && std::isdigit(json[pos])) {
                number += json[pos++];
            }
            
            if (!number.empty() && number != "-") {
                return std::stoi(number);
            }
        }
        return 0;
    }

    std::string serveImage(const std::vector<uchar>& buffer, const std::string& contentType = "image/jpeg") {
        std::cout << "serveImage called with buffer size: " << buffer.size() << std::endl;
        
        if (buffer.empty()) {
            std::cout << "WARNING: Empty buffer, returning 204 No Content" << std::endl;
            return "HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n";
        }
        
        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\n";
        response << "Content-Type: " << contentType << "\r\n";
        response << "Content-Length: " << buffer.size() << "\r\n";
        response << "Cache-Control: no-cache, no-store, must-revalidate\r\n";
        response << "Pragma: no-cache\r\n";
        response << "Expires: 0\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "Connection: close\r\n\r\n";
        
        std::string responseStr = response.str();
        std::cout << "HTTP headers size: " << responseStr.size() << " bytes" << std::endl;
        
        responseStr.insert(responseStr.end(), buffer.begin(), buffer.end());
        
        std::cout << "Total response size: " << responseStr.size() << " bytes" << std::endl;
        return responseStr;
    }

    // Simple image generation functions
    std::vector<uchar> getOriginalImage() {
        std::vector<uchar> buffer;
        cv::Mat frame;
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty()) {
                frame = latest_frame.clone();
            }
        }
        
        if (frame.empty()) {
            std::cout << "No frame available, creating placeholder" << std::endl;
            frame = cv::Mat(240, 320, CV_8UC3, cv::Scalar(50, 50, 50));
            cv::putText(frame, "No Camera Feed", cv::Point(80, 120), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        } else {
            cv::putText(frame, "LIVE CAMERA", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        bool success = cv::imencode(".jpg", frame, buffer, params);
        
        std::cout << "Original image encoded: " << buffer.size() << " bytes, success: " << success << std::endl;
        return buffer;
    }

    std::vector<uchar> getMaskImage() {
        std::vector<uchar> buffer;
        cv::Mat frame, mask;
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty()) {
                frame = latest_frame.clone();
            }
        }
        
        if (frame.empty()) {
            mask = cv::Mat::zeros(240, 320, CV_8UC1);
        } else {
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, hsv_lower, hsv_upper, mask);
        }
        
        // Convert to color for display
        cv::Mat colored_mask;
        cv::cvtColor(mask, colored_mask, cv::COLOR_GRAY2BGR);
        
        int nonZeroPixels = cv::countNonZero(mask);
        std::ostringstream info;
        info << "Mask pixels: " << nonZeroPixels;
        cv::putText(colored_mask, info.str(), cv::Point(10, 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", colored_mask, buffer, params);
        
        std::cout << "Mask image encoded: " << buffer.size() << " bytes" << std::endl;
        return buffer;
    }

    std::vector<uchar> getResultImage() {
        std::vector<uchar> buffer;
        cv::Mat frame, result;
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty()) {
                frame = latest_frame.clone();
            }
        }
        
        if (frame.empty()) {
            result = cv::Mat(240, 320, CV_8UC3, cv::Scalar(50, 50, 50));
            cv::putText(result, "No Detection", cv::Point(90, 120), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        } else {
            result = frame.clone();
            
            // Simple detection visualization
            cv::Mat hsv, mask;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, hsv_lower, hsv_upper, mask);
            
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            int detectedCount = 0;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area < min_area || area > max_area) continue;
                
                double perimeter = cv::arcLength(contour, true);
                double circularity = 4 * CV_PI * area / (perimeter * perimeter);
                
                if (circularity > min_circularity) {
                    cv::Moments m = cv::moments(contour);
                    if (m.m00 > 0) {
                        cv::Point center(m.m10/m.m00, m.m01/m.m00);
                        
                        cv::circle(result, center, 15, cv::Scalar(0, 0, 255), 3);
                        cv::circle(result, center, 5, cv::Scalar(255, 255, 255), -1);
                        
                        std::ostringstream info;
                        info << "A:" << (int)area;
                        cv::putText(result, info.str(), cv::Point(center.x-20, center.y-25),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
                        
                        detectedCount++;
                    }
                }
            }
            
            std::ostringstream info;
            info << "Detected: " << detectedCount << " objects";
            cv::putText(result, info.str(), cv::Point(10, 20), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", result, buffer, params);
        
        std::cout << "Result image encoded: " << buffer.size() << " bytes" << std::endl;
        return buffer;
    }

public:
    FastPuckTracker(int port = 8080) : port(port), running(false) {
        #ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
        #endif
        
        last_fps_update = std::chrono::steady_clock::now();
        std::cout << "Fast Puck Tracker initialized" << std::endl;
    }

    ~FastPuckTracker() {
        stop();
        #ifdef _WIN32
        WSACleanup();
        #endif
    }

    bool initCamera(int camera_id = 0) {
        std::cout << "Trying camera " << camera_id << "..." << std::endl;
        cap.open(camera_id);
        
        if (!cap.isOpened()) {
            std::cout << "Camera " << camera_id << " failed, trying alternatives..." << std::endl;
            
            for (int i = 0; i < 4; i++) {
                if (i == camera_id) continue;
                
                std::cout << "Trying camera " << i << "..." << std::endl;
                cap.open(i);
                if (cap.isOpened()) {
                    std::cout << "Success with camera " << i << "!" << std::endl;
                    break;
                }
            }
        }
        
        if (!cap.isOpened()) {
            std::cerr << "Cannot open any camera!" << std::endl;
            return false;
        }
        
        double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        std::cout << "Camera resolution: " << actual_width << "x" << actual_height << std::endl;
        
        cv::Mat test_frame;
        bool capture_success = cap.read(test_frame);
        if (!capture_success || test_frame.empty()) {
            std::cerr << "Cannot capture test frame from camera!" << std::endl;
            return false;
        }
        
        std::cout << "Camera initialized successfully!" << std::endl;
        return true;
    }

    PuckData detectPuckFast(const cv::Mat& frame) {
        PuckData puck;
        
        if (frame.empty()) return puck;
        
        try {
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            
            cv::Mat mask;
            cv::inRange(hsv, hsv_lower, hsv_upper, mask);
            
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            double best_score = 0;
            cv::Point2f best_center(-1, -1);
            
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area < min_area || area > max_area) continue;
                
                double perimeter = cv::arcLength(contour, true);
                double circularity = 4 * CV_PI * area / (perimeter * perimeter);
                
                if (circularity > min_circularity && area > best_score) {
                    best_score = area;
                    
                    cv::Moments m = cv::moments(contour);
                    if (m.m00 > 0) {
                        best_center.x = m.m10 / m.m00;
                        best_center.y = m.m01 / m.m00;
                        puck.detected = true;
                    }
                }
            }
            
            if (puck.detected) {
                puck.x = best_center.x;
                puck.y = best_center.y;
                
                if (previous_puck.detected) {
                    float dt = (puck.timestamp - previous_puck.timestamp) / 1000.0f;
                    if (dt > 0 && dt < 0.2f) {
                        puck.velocity_x = (puck.x - previous_puck.x) / dt;
                        puck.velocity_y = (puck.y - previous_puck.y) / dt;
                        puck.speed = std::sqrt(puck.velocity_x*puck.velocity_x + puck.velocity_y*puck.velocity_y);
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cout << "Detection error: " << e.what() << std::endl;
        }
        
        return puck;
    }

    void fastCaptureLoop() {
        std::cout << "Starting capture loop..." << std::endl;
        
        while (running) {
            cv::Mat frame;
            if (!cap.read(frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            
            // Update latest frame for serving
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = frame.clone();
            }
            
            // Detection
            PuckData puck = detectPuckFast(frame);
            
            // Thread-safe update
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                previous_puck = current_puck;
                current_puck = puck;
                
                if (puck.detected) {
                    trajectory.push_back(puck);
                    if (trajectory.size() > MAX_TRAJECTORY_POINTS) {
                        trajectory.pop_front();
                    }
                }
            }
            
            // FPS counting
            frame_counter++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_update);
            
            if (elapsed.count() >= 1000) {
                processing_fps = frame_counter.load() / (elapsed.count() / 1000.0);
                frame_counter = 0;
                last_fps_update = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    }

    std::string generateDataJSON() {
        std::ostringstream json;
        
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            
            double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            
            json << "{\n";
            json << "  \"fps\": " << std::fixed << std::setprecision(1) << processing_fps.load() << ",\n";
            json << "  \"timestamp\": " << current_puck.timestamp << ",\n";
            json << "  \"table\": {\n";
            json << "    \"width\": " << (int)actual_width << ",\n";
            json << "    \"height\": " << (int)actual_height << "\n";
            json << "  },\n";
            json << "  \"puck\": {\n";
            json << "    \"detected\": " << (current_puck.detected ? "true" : "false") << ",\n";
            json << "    \"x\": " << current_puck.x << ",\n";
            json << "    \"y\": " << current_puck.y << ",\n";
            json << "    \"velocity_x\": " << current_puck.velocity_x << ",\n";
            json << "    \"velocity_y\": " << current_puck.velocity_y << ",\n";
            json << "    \"speed\": " << current_puck.speed << "\n";
            json << "  },\n";
            json << "  \"trajectory\": [\n";
            
            for (size_t i = 0; i < trajectory.size(); i++) {
                const auto& point = trajectory[i];
                json << "    {\"x\": " << point.x << ", \"y\": " << point.y 
                     << ", \"t\": " << point.timestamp << "}";
                if (i < trajectory.size() - 1) json << ",";
                json << "\n";
            }
            
            json << "  ]\n";
            json << "}\n";
        }
        
        return json.str();
    }

    std::string getOptimizedHTML() {
        return R"HTML(<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Fast Puck Tracker</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #1a1a1a;
            color: white;
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        
        #gameTable {
            border: 3px solid #00ff00;
            background: #000;
            border-radius: 10px;
            margin: 20px;
            box-shadow: 0 0 20px #00ff0050;
        }
        
        .stats {
            display: flex;
            gap: 30px;
            margin: 20px;
            font-size: 18px;
        }
        
        .stat-item {
            background: rgba(255,255,255,0.1);
            padding: 10px 20px;
            border-radius: 8px;
            border-left: 4px solid #00ff00;
        }
        
        .controls {
            margin: 20px;
        }
        
        button {
            background: #ff4444;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 16px;
            margin: 0 10px;
        }
        
        button:hover {
            background: #ff6666;
        }
        
        #fps {
            color: #00ff00;
            font-weight: bold;
        }
        
        .title {
            font-size: 24px;
            margin-bottom: 10px;
            color: #00ff00;
        }
        
        .calibration-images {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 20px;
            margin: 20px;
            max-width: 1200px;
        }
        
        .image-container {
            text-align: center;
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 10px;
        }
        
        .image-container h3 {
            margin-top: 0;
            color: #00ff00;
        }
        
        .image-container img {
            width: 100%;
            max-width: 320px;
            border: 2px solid #333;
            border-radius: 5px;
        }
    </style>
</head>
<body>
    <div class="title">🚀 Fast Puck Tracker</div>
    
    <canvas id="gameTable" width="640" height="480"></canvas>
    
    <div class="stats">
        <div class="stat-item">
            FPS: <span id="fps">--</span>
        </div>
        <div class="stat-item">
            Status: <span id="status">Initializing...</span>
        </div>
        <div class="stat-item">
            Position: <span id="position">--</span>
        </div>
        <div class="stat-item">
            Speed: <span id="speed">--</span>
        </div>
        <div class="stat-item">
            Points: <span id="points">0</span>
        </div>
    </div>
    
    <div class="controls">
        <button onclick="resetTracking()">Reset Tracking</button>
        <button onclick="togglePause()">Pause/Resume</button>
        <button onclick="toggleCalibration()">Toggle Calibration</button>
    </div>
    
    <div id="calibrationPanel" class="calibration-images" style="display: none;">
        <div class="image-container">
            <h3>📹 Original Camera</h3>
            <img id="originalImg" src="/calibration/original" alt="Original camera feed">
        </div>
        
        <div class="image-container">
            <h3>🎨 HSV Mask</h3>
            <img id="maskImg" src="/calibration/mask" alt="HSV mask">
        </div>
        
        <div class="image-container">
            <h3>🎯 Detection Result</h3>
            <img id="resultImg" src="/calibration/result" alt="Detection result">
        </div>
    </div>

    <script>
        const canvas = document.getElementById('gameTable');
        const ctx = canvas.getContext('2d');
        
        let isPaused = false;
        let showCalibration = false;
        let currentData = null;
        
        function drawTable(data) {
            if (!data || isPaused) return;
            
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            ctx.fillStyle = '#003300';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.setLineDash([10, 10]);
            ctx.beginPath();
            ctx.moveTo(canvas.width/2, 0);
            ctx.lineTo(canvas.width/2, canvas.height);
            ctx.stroke();
            ctx.setLineDash([]);
            
            if (data.trajectory && data.trajectory.length > 1) {
                ctx.strokeStyle = '#ffff00';
                ctx.lineWidth = 3;
                ctx.beginPath();
                
                const scaleX = canvas.width / data.table.width;
                const scaleY = canvas.height / data.table.height;
                
                for (let i = 0; i < data.trajectory.length; i++) {
                    const point = data.trajectory[i];
                    const x = point.x * scaleX;
                    const y = point.y * scaleY;
                    
                    if (i === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                    
                    ctx.fillStyle = `rgba(255, 255, 0, ${0.3 + 0.7 * i / data.trajectory.length})`;
                    ctx.beginPath();
                    ctx.arc(x, y, 3, 0, 2 * Math.PI);
                    ctx.fill();
                }
                ctx.stroke();
            }
            
            if (data.puck && data.puck.detected) {
                const scaleX = canvas.width / data.table.width;
                const scaleY = canvas.height / data.table.height;
                const x = data.puck.x * scaleX;
                const y = data.puck.y * scaleY;
                
                ctx.fillStyle = '#ff0000';
                ctx.beginPath();
                ctx.arc(x, y, 15, 0, 2 * Math.PI);
                ctx.fill();
                
                ctx.fillStyle = '#ffffff';
                ctx.beginPath();
                ctx.arc(x, y, 5, 0, 2 * Math.PI);
                ctx.fill();
                
                if (data.puck.speed > 10) {
                    const vx = data.puck.velocity_x * 0.5 * scaleX;
                    const vy = data.puck.velocity_y * 0.5 * scaleY;
                    
                    ctx.strokeStyle = '#00ffff';
                    ctx.lineWidth = 3;
                    ctx.beginPath();
                    ctx.moveTo(x, y);
                    ctx.lineTo(x + vx, y + vy);
                    ctx.stroke();
                    
                    const angle = Math.atan2(vy, vx);
                    ctx.beginPath();
                    ctx.moveTo(x + vx, y + vy);
                    ctx.lineTo(x + vx - 10*Math.cos(angle-0.3), y + vy - 10*Math.sin(angle-0.3));
                    ctx.moveTo(x + vx, y + vy);
                    ctx.lineTo(x + vx - 10*Math.cos(angle+0.3), y + vy - 10*Math.sin(angle+0.3));
                    ctx.stroke();
                }
            }
        }
        
        function updateStats(data) {
            if (!data) return;
            
            document.getElementById('fps').textContent = data.fps.toFixed(1);
            document.getElementById('status').textContent = data.puck.detected ? 'DETECTED' : 'Searching...';
            
            if (data.puck.detected) {
                document.getElementById('position').textContent = 
                    `(${data.puck.x.toFixed(0)}, ${data.puck.y.toFixed(0)})`;
                document.getElementById('speed').textContent = 
                    `${data.puck.speed.toFixed(1)} px/s`;
            } else {
                document.getElementById('position').textContent = '--';
                document.getElementById('speed').textContent = '--';
            }
            
            document.getElementById('points').textContent = data.trajectory.length;
        }
        
        function fetchData() {
            if (isPaused) return;
            
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    currentData = data;
                    drawTable(data);
                    updateStats(data);
                })
                .catch(err => console.error('Data fetch failed:', err));
        }
        
        function resetTracking() {
            fetch('/reset', { method: 'POST' })
                .then(() => console.log('Tracking reset'))
                .catch(err => console.error('Reset failed:', err));
        }
        
        function togglePause() {
            isPaused = !isPaused;
            if (!isPaused && currentData) {
                drawTable(currentData);
            }
        }
        
        function toggleCalibration() {
            showCalibration = !showCalibration;
            const panel = document.getElementById('calibrationPanel');
            panel.style.display = showCalibration ? 'grid' : 'none';
            
            if (showCalibration) {
                refreshCalibrationImages();
            }
        }
        
        function refreshCalibrationImages() {
            if (!showCalibration) return;
            
            const timestamp = new Date().getTime();
            document.getElementById('originalImg').src = '/calibration/original?' + timestamp;
            document.getElementById('maskImg').src = '/calibration/mask?' + timestamp;
            document.getElementById('resultImg').src = '/calibration/result?' + timestamp;
        }

        setInterval(fetchData, 16);
        setInterval(refreshCalibrationImages, 200);
        
        fetchData();
    </script>
</body>
</html>)HTML";
    }

    std::string handleRequest(const std::string& request) {
        std::istringstream iss(request);
        std::string method, path, version;
        iss >> method >> path >> version;

        // Remove query parameters for path matching
        std::string clean_path = path;
        size_t query_pos = clean_path.find('?');
        if (query_pos != std::string::npos) {
            clean_path = clean_path.substr(0, query_pos);
        }

        std::cout << "Request: " << method << " " << path << " -> clean: " << clean_path << std::endl;

        if (method == "GET" && clean_path == "/") {
            std::string html = getOptimizedHTML();
            std::ostringstream response;
            response << "HTTP/1.1 200 OK\r\n";
            response << "Content-Type: text/html; charset=UTF-8\r\n";
            response << "Content-Length: " << html.length() << "\r\n";
            response << "Connection: close\r\n\r\n";
            response << html;
            return response.str();
        }
        else if (method == "GET" && clean_path == "/data") {
            std::string json = generateDataJSON();
            std::ostringstream response;
            response << "HTTP/1.1 200 OK\r\n";
            response << "Content-Type: application/json\r\n";
            response << "Content-Length: " << json.length() << "\r\n";
            response << "Cache-Control: no-cache\r\n";
            response << "Access-Control-Allow-Origin: *\r\n";
            response << "Connection: close\r\n\r\n";
            response << json;
            return response.str();
        }
        else if (method == "POST" && clean_path == "/reset") {
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                trajectory.clear();
                current_puck = PuckData();
                previous_puck = PuckData();
            }
            return "HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nOK";
        }
        // CALIBRATION ENDPOINTS
        else if (method == "GET" && clean_path == "/calibration/original") {
            std::cout << "=== SERVING ORIGINAL IMAGE ===" << std::endl;
            auto buffer = getOriginalImage();
            return serveImage(buffer);
        }
        else if (method == "GET" && clean_path == "/calibration/mask") {
            std::cout << "=== SERVING MASK IMAGE ===" << std::endl;
            auto buffer = getMaskImage();
            return serveImage(buffer);
        }
        else if (method == "GET" && clean_path == "/calibration/result") {
            std::cout << "=== SERVING RESULT IMAGE ===" << std::endl;
            auto buffer = getResultImage();
            return serveImage(buffer);
        }
        else if (method == "POST" && clean_path == "/calibration/params") {
            size_t json_start = request.find("\r\n\r\n");
            if (json_start != std::string::npos) {
                std::string json_body = request.substr(json_start + 4);
                
                int hue_min = extractJSONInt(json_body, "hue_min");
                int hue_max = extractJSONInt(json_body, "hue_max");
                int sat_min = extractJSONInt(json_body, "sat_min");
                int sat_max = extractJSONInt(json_body, "sat_max");
                int val_min = extractJSONInt(json_body, "val_min");
                int val_max = extractJSONInt(json_body, "val_max");
                int min_area_val = extractJSONInt(json_body, "min_area");
                int max_area_val = extractJSONInt(json_body, "max_area");
                int min_circ = extractJSONInt(json_body, "min_circularity");
                
                // Apply parameters
                hsv_lower = cv::Scalar(hue_min, sat_min, val_min);
                hsv_upper = cv::Scalar(hue_max, sat_max, val_max);
                min_area = min_area_val;
                max_area = max_area_val;
                min_circularity = min_circ / 100.0;
                
                std::cout << "Updated detection parameters" << std::endl;
            }
            
            return "HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nOK";
        }

        // 404 Not Found
        std::string notFound = "404 Not Found";
        std::ostringstream response;
        response << "HTTP/1.1 404 Not Found\r\n";
        response << "Content-Type: text/plain\r\n";
        response << "Content-Length: " << notFound.length() << "\r\n";
        response << "Connection: close\r\n\r\n";
        response << notFound;
        return response.str();
    }

    void handleClient(int client_socket) {
        const int BUFFER_SIZE = 8192;
        std::vector<char> buffer(BUFFER_SIZE);
        std::string request;
        
        while (true) {
            int bytes_received = recv(client_socket, buffer.data(), BUFFER_SIZE - 1, 0);
            if (bytes_received <= 0) break;
            
            buffer[bytes_received] = '\0';
            request.append(buffer.data(), bytes_received);
            
            if (request.find("\r\n\r\n") != std::string::npos) {
                break;
            }
        }
        
        if (!request.empty()) {
            std::string response = handleRequest(request);
            
            size_t totalSent = 0;
            const size_t CHUNK_SIZE = 8192;
            
            while (totalSent < response.length()) {
                size_t remaining = response.length() - totalSent;
                size_t chunkSize = std::min(CHUNK_SIZE, remaining);
                
                int bytesSent = send(client_socket, 
                                    response.data() + totalSent, 
                                    chunkSize, 
                                    0);
                
                if (bytesSent <= 0) {
                    std::cerr << "Send failed after " << totalSent << " bytes" << std::endl;
                    break;
                }
                totalSent += bytesSent;
            }
            
            if (totalSent == response.length()) {
                std::cout << "Successfully sent " << totalSent << " bytes" << std::endl;
            }
        }
        
        closesocket(client_socket);
    }

    void serverLoop() {
        std::cout << "Fast HTTP server running on port " << port << std::endl;
        std::cout << "Open browser: http://localhost:" << port << std::endl;
        
        while (running) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_socket = accept(server_socket, (sockaddr*)&client_addr, &client_len);
            if (client_socket < 0) {
                if (running) {
                    std::cerr << "accept() error" << std::endl;
                }
                continue;
            }
            
            std::thread([this, client_socket]() {
                handleClient(client_socket);
            }).detach();
        }
    }

    bool start() {
        if (!initCamera()) {
            return false;
        }

        server_socket = socket(AF_INET, SOCK_STREAM, 0);

        if (server_socket < 0) {
            std::cerr << "Cannot create socket" << std::endl;
            return false;
        }

        int opt = 1;
        setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);

        if (bind(server_socket, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Cannot bind socket" << std::endl;
            closesocket(server_socket);
            return false;
        }

        if (listen(server_socket, 10) < 0) {
            std::cerr << "listen() error" << std::endl;
            closesocket(server_socket);
            return false;
        }

        running = true;
        
        capture_thread = std::thread(&FastPuckTracker::fastCaptureLoop, this);
        server_thread = std::thread(&FastPuckTracker::serverLoop, this);
        
        return true;
    }

    void stop() {
        if (running) {
            running = false;
            
            if (capture_thread.joinable()) {
                capture_thread.join();
            }
            
            if (server_socket >= 0) {
                closesocket(server_socket);
            }
            
            if (server_thread.joinable()) {
                server_thread.join();
            }
            
            if (cap.isOpened()) {
                cap.release();
            }
        }
    }

    void waitForShutdown() {
        std::cout << "Press Enter to stop server..." << std::endl;
        std::cin.get();
        stop();
    }
};

int main(int argc, char* argv[]) {
    int port = 8080;
    int camera_id = 0;
    
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-p" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "-c" && i + 1 < argc) {
            camera_id = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help") {
            std::cout << "Fast Puck Tracker - Working Version" << std::endl;
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "  -p PORT      HTTP server port (default: 8080)" << std::endl;
            std::cout << "  -c ID        Camera ID (default: 0)" << std::endl;
            std::cout << "  -h, --help   Show this help" << std::endl;
            return 0;
        }
    }

    try {
        FastPuckTracker tracker(port);
        
        if (!tracker.start()) {
            std::cerr << "Failed to start tracker!" << std::endl;
            return 1;
        }
        
        std::cout << "\n🚀 Fast Puck Tracker Started!" << std::endl;
        std::cout << "Port: " << port << std::endl;
        std::cout << "URL: http://localhost:" << port << std::endl;
        std::cout << "Click 'Toggle Calibration' to see live images!" << std::endl;
        std::cout << "==============================" << std::endl;
        
        tracker.waitForShutdown();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}