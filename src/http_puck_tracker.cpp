// http_puck_tracker.cpp
// Serwer HTTP z podglądem MJPEG, detekcją krążka, sterowaniem ABB (opcjonalnie)
// oraz kalibracją stołu na 4 punkty (homografia) + overlay pola gry.
//
// Kompilacja z ABB:   -DUSE_ABB i dołączony nagłówek abb_comm.h
// Bez ABB (symulacja): bez USE_ABB

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <atomic>
#include <mutex>
#include <map>
#include <fstream>
#include <chrono>
#include <deque>
#include <cmath>
#include <algorithm>
#include <cctype>

#ifdef USE_ABB
#include "abb_comm.h"   // AbbRobotController, RobotPosition + komendy abb_comm::
#endif

// ======================== Globalna flaga pracy serwera ========================
std::atomic<bool> running{true};

// ======= Puck Detection & Tracking =======
struct PuckPosition {
    cv::Point2f position{};
    float radius{0};
    std::chrono::high_resolution_clock::time_point timestamp{};
    float confidence{0};
};

struct RobotCalibration {
    // Offset robota w [mm] dla (0,0) stołu
    cv::Point3f robot_origin = cv::Point3f(0, 0, 50); // domyślnie 50mm nad stołem
    bool is_calibrated = false;
    std::chrono::high_resolution_clock::time_point calibration_time;
};

struct CalibrationData {
    cv::Scalar hsv_lower = cv::Scalar(0, 100, 100);    // Default dla czerwonego
    cv::Scalar hsv_upper = cv::Scalar(10, 255, 255);
    int   min_area = 100;
    int   max_area = 5000;
    float min_circularity = 0.7f;
    bool  use_background_subtraction = false;
    cv::Rect table_roi = cv::Rect(0, 0, 640, 480);

    // Fallback stary system (gdy brak homografii)
    float pixels_per_meter = 1000.0f;
    cv::Point2f table_center = cv::Point2f(320, 240);
};

struct TrajectoryPredictor {
    std::deque<PuckPosition> history;
    cv::Point2f velocity = cv::Point2f(0, 0);
    cv::Point2f predicted_position = cv::Point2f(0, 0);
    float bounce_coefficient = 0.8f;
    int   max_history = 10;

    void addPosition(const PuckPosition& pos) {
        history.push_back(pos);
        if ((int)history.size() > max_history) history.pop_front();
        updateVelocity();
        predictTrajectory();
    }
    void updateVelocity() {
        if (history.size() < 2) return;
        const auto& c = history.back();
        const auto& p = history[history.size() - 2];
        float dt = std::chrono::duration<float>(c.timestamp - p.timestamp).count();
        if (dt > 0.f) {
            velocity.x = (c.position.x - p.position.x) / dt;
            velocity.y = (c.position.y - p.position.y) / dt;
        }
    }
    void predictTrajectory() {
        if (history.empty()) return;
        float dt = 0.1f;
        predicted_position = history.back().position + velocity * dt;
        if (predicted_position.x < 0 || predicted_position.x > 640) velocity.x *= -bounce_coefficient;
        if (predicted_position.y < 0 || predicted_position.y > 480) velocity.y *= -bounce_coefficient;
    }
};

// ======= Kalibracja stołu (homografia 4 punktów) =======
struct TableCalibration {
    // 4 narożniki w obrazie (px) w kolejności kliknięcia
    std::vector<cv::Point2f> img_pts;   // size == 4

    // Wymiary realne stołu (mm)
    float width_mm  = 1000.0f;
    float height_mm = 500.0f;

    // Który narożnik jest (0,0) (0..3 wg kolejności kliknięć)
    int  origin_index = 0;
    // Kierunek osi (czy idziemy po rogach zgodnie z ruchem wskazówek zegara)
    bool clockwise = true;

    // Homografia: piksele -> [mm]
    cv::Mat H_pix2tab;   // 3x3
    cv::Mat H_tab2pix;   // 3x3
    bool ready = false;

    // Zwraca 4 punkty docelowe (mm) zgodnie z origin_index i clockwise
    std::vector<cv::Point2f> table_corners_mm() const {
        cv::Point2f P0(0.0f,        0.0f);
        cv::Point2f P1(width_mm,    0.0f);
        cv::Point2f P2(width_mm, height_mm);
        cv::Point2f P3(0.0f,   height_mm);
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

    // piksele -> metry w układzie stołu
    cv::Point2f pixelToTableMeters(const cv::Point2f& p) const {
        if (!ready) return {0.f, 0.f};
        std::vector<cv::Point2f> src{p}, dst;
        cv::perspectiveTransform(src, dst, H_pix2tab);
        return { dst[0].x / 1000.f, dst[0].y / 1000.f };
    }

    // [mm] stołu -> piksele (do rysowania)
    cv::Point2f tableToPixel_mm(const cv::Point2f& t_mm) const {
        if (!ready) return {0.f, 0.f};
        std::vector<cv::Point2f> src{t_mm}, dst;
        cv::perspectiveTransform(src, dst, H_tab2pix);
        return dst[0];
    }
};

// ======= Robot Control =======
struct RobotCommand {
    float x{0}, y{0}, z{0};   // metry
    float rotation{0};        // nieużywane
    float speed{0.5f};        // 0..1
    bool  execute_strike{false};
};

struct GameStrategy {
    enum Mode { DEFENSIVE, OFFENSIVE, INTERCEPT };
    Mode current_mode = DEFENSIVE;
    cv::Point2f robot_position = cv::Point2f(0, 0); // m, w układzie stołu
    float strike_threshold_m = 0.05f; // 5 cm
};

// ======= Global State =======
CalibrationData      g_calibration;
TrajectoryPredictor  g_predictor;
GameStrategy         g_strategy;
std::mutex           g_detection_mutex;
PuckPosition         g_current_puck;
cv::Ptr<cv::BackgroundSubtractor> g_bg_subtractor;
RobotCalibration     g_robot_calibration;
TableCalibration     g_table_calib;

// ======= Robot diagnostics =======
struct RobotDiag {
    std::string ip = "192.168.125.1";
    uint16_t    port = 11000; // motion port (SERVER.mod)
    std::string last_error;
} g_robot_diag;

std::mutex g_robot_mx;

#ifdef USE_ABB
static AbbRobotController* g_robot = nullptr;
#endif

// ======= Utils =======
static std::string escape_json(const std::string& s) {
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

// prosty parser liczb z JSON
static std::vector<double> parse_numbers(const std::string& body) {
    std::vector<double> nums; std::string cur;
    auto flush=[&]{ if(!cur.empty()){ try{ nums.push_back(std::stod(cur)); }catch(...){} cur.clear(); } };
    for(char c: body){
        if(std::isdigit((unsigned char)c) || c=='-'||c=='+'||c=='.'||c=='e'||c=='E')
            cur.push_back(c);
        else flush();
    }
    flush();
    return nums;
}

// ======= Puck Detection =======
bool detectPuckColor(const cv::Mat& frame, PuckPosition& result) {
    cv::Mat hsv, mask, mask_filtered;
    cv::cvtColor(frame(g_calibration.table_roi), hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, g_calibration.hsv_lower, g_calibration.hsv_upper, mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask_filtered, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask_filtered, mask_filtered, cv::MORPH_CLOSE, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_filtered, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    float best_score = 0;
    cv::Point2f best_center; float best_radius = 0;
    for (const auto& contour : contours) {
        float area = (float)cv::contourArea(contour);
        if (area < g_calibration.min_area || area > g_calibration.max_area) continue;
        float perimeter = std::max(1.0f, (float)cv::arcLength(contour, true));
        float circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < g_calibration.min_circularity) continue;
        cv::Point2f center; float radius;
        cv::minEnclosingCircle(contour, center, radius);
        center.x += (float)g_calibration.table_roi.x;
        center.y += (float)g_calibration.table_roi.y;
        float score = area * circularity;
        if (score > best_score) { best_score = score; best_center = center; best_radius = radius; }
    }
    if (best_score > 0) {
        result.position = best_center;
        result.radius = best_radius;
        result.timestamp = std::chrono::high_resolution_clock::now();
        result.confidence = std::min(1.0f, best_score / 10000.0f);
        return true;
    }
    return false;
}

bool detectPuckBackground(const cv::Mat& frame, PuckPosition& result) {
    if (!g_bg_subtractor) g_bg_subtractor = cv::createBackgroundSubtractorMOG2(500, 16, false);
    cv::Mat fg_mask, fg_filtered;
    g_bg_subtractor->apply(frame(g_calibration.table_roi), fg_mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(fg_mask, fg_filtered, cv::MORPH_OPEN, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_filtered, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    float best_score = 0; cv::Point2f best_center; float best_radius = 0;
    for (const auto& contour : contours) {
        float area = (float)cv::contourArea(contour);
        if (area < g_calibration.min_area || area > g_calibration.max_area) continue;
        cv::Point2f center; float radius;
        cv::minEnclosingCircle(contour, center, radius);
        center.x += (float)g_calibration.table_roi.x;
        center.y += (float)g_calibration.table_roi.y;
        if (area > best_score) { best_score = area; best_center = center; best_radius = radius; }
    }
    if (best_score > 0) {
        result.position = best_center;
        result.radius = best_radius;
        result.timestamp = std::chrono::high_resolution_clock::now();
        result.confidence = std::min(1.0f, best_score / 5000.0f);
        return true;
    }
    return false;
}

bool detectPuck(const cv::Mat& frame, PuckPosition& result) {
    if (g_calibration.use_background_subtraction) return detectPuckBackground(frame, result);
    return detectPuckColor(frame, result);
}

// ======= Robot Command Synthesis (z kalibracją stołu) =======
RobotCommand calculateRobotCommandCalibrated(const PuckPosition& puck, const TrajectoryPredictor& predictor) {
    RobotCommand cmd{};
    cmd.speed = 0.5f;
    cmd.z     = 0.05f; // 5 cm nad stołem

    // Używaj układu stołu (metry) – z homografią jeśli gotowa, inaczej fallback wokół środka
    cv::Point2f puck_m, pred_m;
    if (g_table_calib.ready) {
        puck_m = g_table_calib.pixelToTableMeters(puck.position);
        pred_m = g_table_calib.pixelToTableMeters(predictor.predicted_position);
    } else {
        // Fallback „na oko”
        puck_m.x = (puck.position.x - g_calibration.table_center.x) / g_calibration.pixels_per_meter;
        puck_m.y = (puck.position.y - g_calibration.table_center.y) / g_calibration.pixels_per_meter;
        pred_m.x = (predictor.predicted_position.x - g_calibration.table_center.x) / g_calibration.pixels_per_meter;
        pred_m.y = (predictor.predicted_position.y - g_calibration.table_center.y) / g_calibration.pixels_per_meter;
    }
    float distance_to_puck = cv::norm(puck_m - g_strategy.robot_position);
    // --- PRZECHWYT / STRIKE ---
    const float BOX_X = 0.03f;     // +/-3 cm w osi X
    const float PRELOAD = 0.02f;   // 2 cm cofnięcia przed kontaktem
    const float MIN_SPEED_PX = 80.0f; // opcjonalnie: jeśli masz prędkość w px/s
    // warunek: wyrównaliśmy X z krążkiem i krążek jest blisko w Y
    if (std::abs(pred_m.x - g_strategy.robot_position.x) < BOX_X &&
        std::abs(pred_m.y - g_strategy.robot_position.y) < 0.06f) // 6 cm w Y
    {
        // ustaw się tuż przed puckiem i uderz
        cmd.x = pred_m.x;
        cmd.y = pred_m.y - PRELOAD; // bronimy dołem; dla góry daj +PRELOAD
        cmd.z = 0.05f;
        cmd.execute_strike = true;
        cmd.speed = 1.0f;
        return cmd;
    }
    

    switch (g_strategy.current_mode) {
        case GameStrategy::DEFENSIVE:
            cmd.x = 0.0f;
            cmd.y = puck_m.y * 0.5f;
            break;
        case GameStrategy::OFFENSIVE:
            cmd.x = pred_m.x + 0.10f;
            cmd.y = pred_m.y;
            break;
        case GameStrategy::INTERCEPT:
            cmd.x = pred_m.x;
            cmd.y = pred_m.y;
            if (distance_to_puck < g_strategy.strike_threshold_m) {
                cmd.execute_strike = true;
                cmd.speed = 1.0f;
            }
            break;
    }

    // Limity bezpieczeństwa robota
    cmd.x = std::max(-0.6f, std::min(0.6f, cmd.x));
    cmd.y = std::max(-0.4f, std::min(0.4f, cmd.y));
    cmd.z = std::max(0.02f, std::min(0.1f,  cmd.z));

    // Debug
    std::cout << "[DBG] puck_px=(" << puck.position.x << "," << puck.position.y
              << ") table_m=(" << puck_m.x << "," << puck_m.y
              << ") cmd=(" << cmd.x << "," << cmd.y << "," << cmd.z
              << ") speed=" << cmd.speed << " strike=" << (cmd.execute_strike?1:0) << "\n";

    return cmd;
}

// ======= Warstwa komunikacji z ABB =======
bool sendRobotCommand(const RobotCommand& cmd) {
#ifdef USE_ABB
    std::lock_guard<std::mutex> lk(g_robot_mx);
    if (!g_robot || !g_robot->isConnected()) return false;

    try {
        double tcp_mm_s  = std::clamp<double>(cmd.speed, 0.05, 1.0) * 500.0; // 25..500 mm/s
        double ori_deg_s = std::clamp<double>(cmd.speed, 0.05, 1.0) * 100.0;

        g_robot->setSpeed(tcp_mm_s, ori_deg_s);

        // Narzędzie pionowo
        double q0=1.0, qx=0.0, qy=0.0, qz=0.0;

        // Ruch
        g_robot->moveCartesian(cmd.x, cmd.y, cmd.z, q0, qx, qy, qz);

        // Strike
                // Impuls „strike” + powrót (w osi Y)
        if (cmd.execute_strike) {
            g_robot->setSpeed(800.0, 120.0);
            // kierunek +Y (jeśli bronisz dołem stołu). Jeśli bronisz górą – daj minus.
            double dash = 0.06; // 6 cm
            double q0=1.0, qx=0.0, qy=0.0, qz=0.0;
        
            double x = cmd.x, y = cmd.y, z = cmd.z;
            double y_hit = std::clamp(y + dash, -0.4, 0.4);
        
            // zejście pre-hit (opcjonalnie: minimalne zbliżenie)
            g_robot->moveCartesian(x, y, z, q0, qx, qy, qz);
        
            // uderzenie
            g_robot->moveCartesian(x, y_hit, z, q0, qx, qy, qz);
        
            // powrót
            g_robot->setSpeed(std::clamp<double>(cmd.speed, 0.05, 1.0)*500.0,
                              std::clamp<double>(cmd.speed, 0.05, 1.0)*100.0);
            g_robot->moveCartesian(x, y, z, q0, qx, qy, qz);
        }
        return true;
    } catch (const std::exception& e) {
        g_robot_diag.last_error = std::string("Command failed: ") + e.what();
        return false;
    }
#else
    // Symulacja: log
    std::cout << "Robot Command: x=" << cmd.x << ", y=" << cmd.y << ", z=" << cmd.z
              << ", speed=" << cmd.speed << ", strike=" << (cmd.execute_strike?"yes":"no") << std::endl;
    return true;
#endif
}

// ======= HTTP helpers =======
void send_http_response(int client_socket, const std::string& content_type, const std::string& content) {
    std::string response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + std::to_string(content.length()) + "\r\n"
        "Connection: close\r\n"
        "\r\n" + content;
    send(client_socket, response.c_str(), response.length(), 0);
}

std::string robot_status_json() {
    std::lock_guard<std::mutex> lk(g_robot_mx);
    bool connected = false;
#ifdef USE_ABB
    connected = (g_robot != nullptr && g_robot->isConnected());
#endif
    std::ostringstream os;
    os << "{"
       << "\"ip\":\"" << escape_json(g_robot_diag.ip) << "\","
       << "\"port\":" << g_robot_diag.port << ","
       << "\"connected\":" << (connected ? "true" : "false") << ","
       << "\"last_error\":\"" << escape_json(g_robot_diag.last_error) << "\""
       << "}";
    return os.str();
}

std::string detection_status_json() {
    std::lock_guard<std::mutex> lk(g_detection_mutex);

    // Pozycje w układzie stołu (m)
    cv::Point2f table_xy  = g_table_calib.ready
                          ? g_table_calib.pixelToTableMeters(g_current_puck.position)
                          : cv::Point2f(
                                (g_current_puck.position.x - g_calibration.table_center.x)/g_calibration.pixels_per_meter,
                                (g_current_puck.position.y - g_calibration.table_center.y)/g_calibration.pixels_per_meter
                            );
    cv::Point2f table_xyP = g_table_calib.ready
                          ? g_table_calib.pixelToTableMeters(g_predictor.predicted_position)
                          : cv::Point2f(
                                (g_predictor.predicted_position.x - g_calibration.table_center.x)/g_calibration.pixels_per_meter,
                                (g_predictor.predicted_position.y - g_calibration.table_center.y)/g_calibration.pixels_per_meter
                            );

    std::ostringstream os;
    os << "{"
       << "\"puck_detected\":" << (g_current_puck.confidence > 0 ? "true" : "false") << ","
       << "\"pixel_position\":{\"x\":" << g_current_puck.position.x << ",\"y\":" << g_current_puck.position.y << "},"
       << "\"table_position_m\":{\"x\":" << table_xy.x << ",\"y\":" << table_xy.y << "},"
       << "\"radius_px\":" << g_current_puck.radius << ","
       << "\"confidence\":" << g_current_puck.confidence << ","
       << "\"predicted_pixel\":{\"x\":" << g_predictor.predicted_position.x << ",\"y\":" << g_predictor.predicted_position.y << "},"
       << "\"predicted_table_m\":{\"x\":" << table_xyP.x << ",\"y\":" << table_xyP.y << "},"
       << "\"velocity_px_per_s\":{\"x\":" << g_predictor.velocity.x << ",\"y\":" << g_predictor.velocity.y << "},"
       << "\"table_ready\":" << (g_table_calib.ready ? "true":"false")
       << "}";
    return os.str();
}

// ======= MJPEG helpers =======
static std::string mat_to_jpeg(const cv::Mat& img, int quality=80) {
    std::vector<uchar> jpeg_buffer;
    cv::imencode(".jpg", img, jpeg_buffer, {cv::IMWRITE_JPEG_QUALITY, quality});
    return std::string(jpeg_buffer.begin(), jpeg_buffer.end());
}

static cv::Mat generate_mask(const cv::Mat& frame) {
    cv::Mat mask;
    if (g_calibration.use_background_subtraction) {
        if (!g_bg_subtractor) g_bg_subtractor = cv::createBackgroundSubtractorMOG2();
        g_bg_subtractor->apply(frame(g_calibration.table_roi), mask);
    } else {
        cv::Mat hsv;
        cv::cvtColor(frame(g_calibration.table_roi), hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, g_calibration.hsv_lower, g_calibration.hsv_upper, mask);
    }
    cv::Mat full_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    mask.copyTo(full_mask(g_calibration.table_roi));
    return full_mask;
}

static cv::Mat generate_result(const cv::Mat& frame, const cv::Mat& mask) {
    cv::Mat colorMask;
    if (mask.channels() == 1) cv::cvtColor(mask, colorMask, cv::COLOR_GRAY2BGR);
    else colorMask = mask;
    cv::Mat result;
    cv::addWeighted(frame, 1.0, colorMask, 0.5, 0, result);
    return result;
}

// Overlay info o robocie
void draw_calibration_overlay(cv::Mat& frame) {
    if (g_robot_calibration.is_calibrated) {
        std::string info = "Robot origin [mm]: (" +
            std::to_string((int)std::round(g_robot_calibration.robot_origin.x)) + ", " +
            std::to_string((int)std::round(g_robot_calibration.robot_origin.y)) + ", " +
            std::to_string((int)std::round(g_robot_calibration.robot_origin.z)) + ")";
        cv::putText(frame, info, cv::Point(10, frame.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    }
}

// Overlay pola gry wg homografii stołu
void draw_table_overlay(cv::Mat& frame) {
    if (!g_table_calib.ready) return;

    // obrys prostokąta stołu
    auto corners_mm = g_table_calib.table_corners_mm();
    std::vector<cv::Point> poly;
    poly.reserve(4);
    for (auto& t : corners_mm) {
        cv::Point2f p = g_table_calib.tableToPixel_mm(t);
        poly.emplace_back(cv::Point((int)std::round(p.x), (int)std::round(p.y)));
    }
    const cv::Scalar col_field(0,255,0);
    for (int i=0;i<4;++i) {
        cv::line(frame, poly[i], poly[(i+1)%4], col_field, 2);
    }

    // osie i origin (0,0) = corners_mm[0]
    cv::Point2f origin_mm = corners_mm[0];
    cv::Point2f ox_mm = origin_mm + cv::Point2f(std::min(100.0f, g_table_calib.width_mm), 0.0f);
    cv::Point2f oy_mm = origin_mm + cv::Point2f(0.0f, std::min(100.0f, g_table_calib.height_mm));

    cv::Point2f o_px  = g_table_calib.tableToPixel_mm(origin_mm);
    cv::Point2f ox_px = g_table_calib.tableToPixel_mm(ox_mm);
    cv::Point2f oy_px = g_table_calib.tableToPixel_mm(oy_mm);

    cv::arrowedLine(frame, o_px, ox_px, cv::Scalar(0,0,255), 2); // X
    cv::arrowedLine(frame, o_px, oy_px, cv::Scalar(255,0,0),  2); // Y
    cv::putText(frame, "0,0", o_px + cv::Point2f(5,-5),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);

    // siatka co 100 mm
    int step = 100;
    cv::Scalar grid(100,100,100);
    for (int y=step; y<=(int)g_table_calib.height_mm; y+=step) {
        cv::Point2f a = g_table_calib.tableToPixel_mm(origin_mm + cv::Point2f(0,(float)y));
        cv::Point2f b = g_table_calib.tableToPixel_mm(origin_mm + cv::Point2f(g_table_calib.width_mm,(float)y));
        cv::line(frame, a, b, grid, 1);
    }
    for (int x=step; x<=(int)g_table_calib.width_mm; x+=step) {
        cv::Point2f a = g_table_calib.tableToPixel_mm(origin_mm + cv::Point2f((float)x,0));
        cv::Point2f b = g_table_calib.tableToPixel_mm(origin_mm + cv::Point2f((float)x,g_table_calib.height_mm));
        cv::line(frame, a, b, grid, 1);
    }
}

void send_mjpeg_stream(int client_socket, cv::VideoCapture& cap) {
    std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Connection: keep-alive\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n";
    send(client_socket, header.c_str(), header.length(), 0);

    cv::Mat frame;
    std::vector<uchar> jpeg_buffer;

    while (running) {
        cap >> frame;
        if (frame.empty()) continue;

        // Detekcja i ewentualne sterowanie
        PuckPosition current_puck{};
        if (detectPuck(frame, current_puck)) {
            {
                std::lock_guard<std::mutex> lk(g_detection_mutex);
                g_current_puck = current_puck;
                g_predictor.addPosition(current_puck);
            }
            // Wysyłamy komendy do robota TYLKO gdy kalibracja stołu jest gotowa
            if (g_table_calib.ready) {
                RobotCommand cmd = calculateRobotCommand(current_puck, g_predictor);
                sendRobotCommand(cmd);
            } else {
                // Opcjonalny log:
                // std::cout << "[CAL] Table not calibrated – skip robot command\n";
            }
        }

        // Overlay
        cv::Mat display_frame = frame.clone();
        cv::rectangle(display_frame, g_calibration.table_roi, cv::Scalar(0, 255, 0), 2);
        if (g_current_puck.confidence > 0) {
            cv::circle(display_frame, g_current_puck.position, (int)std::round(g_current_puck.radius), cv::Scalar(0, 0, 255), 2);
            cv::circle(display_frame, g_current_puck.position, 3, cv::Scalar(0, 0, 255), -1);
            cv::circle(display_frame, g_predictor.predicted_position, 5, cv::Scalar(255, 0, 0), 2);
            if (g_predictor.history.size() > 1) {
                for (size_t i = 1; i < g_predictor.history.size(); ++i) {
                    cv::line(display_frame, g_predictor.history[i-1].position,
                             g_predictor.history[i].position, cv::Scalar(255, 255, 0), 1);
                }
            }
            cv::Point2f vel_end = g_current_puck.position + g_predictor.velocity * 0.1f;
            cv::arrowedLine(display_frame, g_current_puck.position, vel_end, cv::Scalar(0, 255, 255), 2);
        }
        draw_calibration_overlay(display_frame);
        draw_table_overlay(display_frame);

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

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

// ======= HTML page =======
std::string get_html_page() {
    return R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Puck Tracker & Robot Control</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin:0; padding:20px; background:#f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; background:#fff; padding:20px; border-radius:10px; box-shadow:0 2px 10px rgba(0,0,0,0.1); }
        h1 { color:#333; text-align:center; }
        .video-container { text-align:center; margin:20px 0; position:relative; }
        img { max-width:100%; border:2px solid #333; border-radius:5px; cursor: crosshair; }
        .controls { text-align:center; margin:20px 0; }
        button { background:#4CAF50; border:none; color:#fff; padding:10px 20px; font-size:16px; margin:4px 2px; cursor:pointer; border-radius:5px; }
        button:hover { background:#45a049; }
        .info { background:#e7f3ff; border-left:6px solid #2196F3; margin: 20px 0; padding:10px; }
        input, select { border:1px solid #ccc; border-radius:6px; padding:8px; margin:2px; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 8px; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .slider-container { margin: 10px 0; }
        .slider-container label { display: inline-block; width: 150px; }
        .pill { display:inline-block; padding:2px 8px; border-radius:12px; background:#ddd; font-size:12px; margin-left:6px; }
        .pill.ok { background:#c8f7c5; }
        .pill.no { background:#ffd5cc; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🏒 Puck Tracker & Robot Control System</h1>

        <div class="status-grid">
            <div class="section">
                <h2>📹 Video Stream <span id="tbl-ready" class="pill">table: ?</span></h2>
                <div class="video-container">
                    <img id="video-stream" src="/stream" alt="Camera Stream" onload="updateStatus('Połączono')" onerror="updateStatus('Błąd połączenia')">
                </div>
                <div class="controls">
                    <button onclick="refreshStream()">🔄 Odśwież</button>
                    <button onclick="takeSnapshot()">📷 Zdjęcie</button>
                    <button onclick="toggleFullscreen()">🖥️ Pełny Ekran</button>
                </div>
            </div>

            <div class="section">
                <h2>🎯 Detekcja Krążka</h2>
                <div id="detection-status" class="info">
                    <p><strong>Status:</strong> <span id="d-detected">—</span></p>
                    <p><strong>Pozycja (px):</strong> <span id="d-position-px">—</span></p>
                    <p><strong>Pozycja (m, stół):</strong> <span id="d-position-m">—</span></p>
                    <p><strong>Prędkość (px/s):</strong> <span id="d-velocity">—</span></p>
                    <p><strong>Pewność:</strong> <span id="d-confidence">—</span></p>
                </div>
            </div>
        </div>

        <div class="section">
            <h2>⚙️ Kalibracja Detekcji (HSV)</h2>
            <div class="controls">
                <select id="detection-method">
                    <option value="color">Detekcja kolorów HSV</option>
                    <option value="background">Odejmowanie tła</option>
                </select>
                <button onclick="toggleDetectionMethod()">Przełącz metodę</button>
                <button onclick="saveCalibration()">💾 Zapisz</button>
                <button onclick="loadCalibration()">📁 Wczytaj</button>
            </div>

            <div id="hsv-controls" style="display:block;">
                <h3>Parametry HSV (czerwony)</h3>
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

            <div class="controls">
                <button onclick="showCalibrationImage('original')">🖼️ Oryginał</button>
                <button onclick="showCalibrationImage('mask')">🎭 Maska</button>
                <button onclick="showCalibrationImage('result')">🎯 Wynik</button>
            </div>
        </div>

        <div class="section">
          <h2>📏 Kalibracja stołu (4 punkty)</h2>
          <p>Kliknij na obrazie <b>4 narożniki stołu</b> (w dowolnej kolejności). Podaj wymiary stołu i wybierz, który kliknięty narożnik ma być (0,0).</p>
          <div class="controls">
            Szerokość [mm]: <input id="tbl-w" type="number" value="1000" style="width:100px">
            Wysokość [mm]:  <input id="tbl-h" type="number" value="500"  style="width:100px">
            Origin:
            <select id="tbl-origin">
              <option value="0">Punkt #1</option>
              <option value="1">Punkt #2</option>
              <option value="2">Punkt #3</option>
              <option value="3">Punkt #4</option>
            </select>
            Kierunek:
            <select id="tbl-cw">
              <option value="1">CW (z ruchem wskazówek)</option>
              <option value="0">CCW</option>
            </select>
          </div>
          <div class="controls">
            <button onclick="startCornerPick()">🎯 Zbieraj narożniki</button>
            <button onclick="resetCorners()">♻️ Reset narożników</button>
            <button onclick="sendTableCalibration()">✅ Zapisz kalibrację stołu</button>
            <button onclick="setRobotOrigin()">📌 Ustaw origin robota (z bieżącej pozycji)</button>
          </div>
          <div class="info">
            Zebrane punkty: <span id="corners-count">0</span> / 4
          </div>
        </div>

        <div class="section">
            <h2>🤖 Robot ABB – Kontrola</h2>
            <div id="robot-status" class="info">
                <p><strong>Status:</strong> <span id="r-connected">—</span></p>
                <p><strong>IP:</strong> <span id="r-ip">—</span> &nbsp; <strong>Port:</strong> <span id="r-port">—</span></p>
                <p><strong>Ostatni błąd:</strong> <span id="r-err">—</span></p>
            </div>
            <div class="controls">
                <input id="ip" type="text" placeholder="IP" value="192.168.125.1" style="padding:8px">
                <input id="port" type="number" placeholder="Port" value="11000" style="padding:8px; width:110px">
                <button onclick="robotConnect()">🔌 Połącz</button>
                <button onclick="robotDisconnect()">❌ Rozłącz</button>
            </div>

            <h3>Strategia gry</h3>
            <div class="controls">
                <select id="game-strategy">
                    <option value="defensive">Defensywa</option>
                    <option value="offensive">Ofensywa</option>
                    <option value="intercept">Przechwytywanie</option>
                </select>
                <button onclick="updateStrategy()">Ustaw strategię</button>
                <button onclick="emergencyStop()">🛑 STOP AWARYJNY</button>
            </div>

            <h3>Ręczne sterowanie</h3>
            <div class="controls">
                <label>X: </label><input type="range" id="manual-x" min="-0.6" max="0.6" step="0.01" value="0">
                <span id="manual-x-val">0.00</span><br>
                <label>Y: </label><input type="range" id="manual-y" min="-0.4" max="0.4" step="0.01" value="0">
                <span id="manual-y-val">0.00</span><br>
                <label>Z: </label><input type="range" id="manual-z" min="0.02" max="0.1" step="0.01" value="0.05">
                <span id="manual-z-val">0.05</span><br>
                <button onclick="sendManualCommand()">Przenieś</button>
                <button onclick="executeStrike()">⚡ Uderz!</button>
            </div>
        </div>
    </div>

    <script>
        // Status update functions
        async function fetchDetectionStatus() {
            try {
                const r = await fetch('/detection/status');
                const j = await r.json();
                document.getElementById('d-detected').textContent = j.puck_detected ? 'Wykryty' : 'Nie wykryty';
                document.getElementById('d-position-px').textContent = `(${j.pixel_position.x.toFixed(1)}, ${j.pixel_position.y.toFixed(1)})`;
                document.getElementById('d-position-m').textContent  = `(${j.table_position_m.x.toFixed(3)}, ${j.table_position_m.y.toFixed(3)})`;
                document.getElementById('d-velocity').textContent    = `(${j.velocity_px_per_s.x.toFixed(1)}, ${j.velocity_px_per_s.y.toFixed(1)})`;
                document.getElementById('d-confidence').textContent  = `${(j.confidence * 100).toFixed(1)}%`;
                const pill = document.getElementById('tbl-ready');
                pill.textContent = j.table_ready ? 'table: ready' : 'table: not ready';
                pill.className = 'pill ' + (j.table_ready ? 'ok' : 'no');
            } catch (e) {
                document.getElementById('d-detected').textContent = 'Błąd';
            }
        }

        async function fetchRobotStatus() {
            try {
                const r = await fetch('/robot/status');
                const j = await r.json();
                document.getElementById('r-connected').textContent = j.connected ? 'Połączony' : 'Niepołączony';
                document.getElementById('r-ip').textContent = j.ip;
                document.getElementById('r-port').textContent = j.port;
                document.getElementById('r-err').textContent = j.last_error || '-';
            } catch (e) {
                document.getElementById('r-connected').textContent = 'Błąd';
                document.getElementById('r-err').textContent = e.toString();
            }
        }

        async function loadCalibrationData() {
            try {
                const r = await fetch('/calibration/settings');
                const j = await r.json();
                document.getElementById('h-min').value = j.hsv_lower[0];
                document.getElementById('s-min').value = j.hsv_lower[1];
                document.getElementById('v-min').value = j.hsv_lower[2];
                document.getElementById('h-max').value = j.hsv_upper[0];
                document.getElementById('s-max').value = j.hsv_upper[1];
                document.getElementById('v-max').value = j.hsv_upper[2];
                document.getElementById('min-area').value = j.min_area;
                document.getElementById('max-area').value = j.max_area;
                updateSliderValues();
            } catch (e) {
                console.error('Failed to load calibration:', e);
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

        function toggleDetectionMethod() {
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

        function saveCalibration() {
            fetch('/calibration/save', {method: 'POST'})
                .then(() => alert('Kalibracja zapisana!'))
                .catch(() => alert('Błąd zapisu kalibracji'));
        }

        function loadCalibration() {
            fetch('/calibration/load', {method: 'POST'})
                .then(() => {
                    loadCalibrationData();
                    alert('Kalibracja wczytana!');
                })
                .catch(() => alert('Błąd wczytywania kalibracji'));
        }

        // --- Kalibracja stołu (klik w obraz)
        let cornerPickEnabled = false;
        let pickedCorners = []; // [{x,y}, ...]

        function startCornerPick() {
            pickedCorners = [];
            document.getElementById('corners-count').textContent = '0';
            cornerPickEnabled = true;
            alert('Kliknij 4 narożniki stołu na podglądzie.');
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

        (function(){
            const img = document.getElementById('video-stream');
            img.addEventListener('click', function(evt){
                if (!cornerPickEnabled) return;
                const p = imageClickToPixel(img, evt);
                pickedCorners.push(p);
                document.getElementById('corners-count').textContent = pickedCorners.length.toString();
                if (pickedCorners.length === 4) {
                    cornerPickEnabled = false;
                    alert('Zebrano 4 punkty. Teraz "Zapisz kalibrację stołu".');
                }
            });
        })();

        async function sendTableCalibration() {
            if (pickedCorners.length !== 4) { alert('Najpierw kliknij 4 narożniki.'); return; }
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
            if (j.ready) alert('Kalibracja stołu OK! Overlay powinien być widoczny na podglądzie.');
            else alert('Kalibracja nieudana.');
        }

        async function setRobotOrigin() {
            const r = await fetch('/calibration/robot_origin', {method:'POST'});
            const j = await r.json();
            alert(j.message || j.status);
        }

        // Robot control functions
        async function robotConnect() {
            const ip = document.getElementById('ip').value.trim();
            const port = document.getElementById('port').value.trim();
            await fetch(`/robot/connect?ip=${encodeURIComponent(ip)}&port=${encodeURIComponent(port)}`, {method:'POST'});
            fetchRobotStatus();
        }
        async function robotDisconnect() {
            await fetch('/robot/disconnect', {method:'POST'});
            fetchRobotStatus();
        }
        function updateStrategy() {
            const strategy = document.getElementById('game-strategy').value;
            fetch('/robot/strategy', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({strategy: strategy})
            });
        }
        function emergencyStop() {
            fetch('/robot/emergency_stop', {method: 'POST'});
        }
        function sendManualCommand() {
            const x = parseFloat(document.getElementById('manual-x').value);
            const y = parseFloat(document.getElementById('manual-y').value);
            const z = parseFloat(document.getElementById('manual-z').value);
            fetch('/robot/manual_move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({x: x, y: y, z: z, speed: 0.5})
            });
        }
        function executeStrike() {
            fetch('/robot/strike', {method: 'POST'});
        }

        // Slider labels
        document.getElementById('manual-x').addEventListener('input', function() {
            document.getElementById('manual-x-val').textContent = parseFloat(this.value).toFixed(2);
        });
        document.getElementById('manual-y').addEventListener('input', function() {
            document.getElementById('manual-y-val').textContent = parseFloat(this.value).toFixed(2);
        });
        document.getElementById('manual-z').addEventListener('input', function() {
            document.getElementById('manual-z-val').textContent = parseFloat(this.value).toFixed(2);
        });

        // Video stream helpers
        function updateStatus(status) {
            const statusEl = document.getElementById('status');
            if (statusEl) statusEl.textContent = status;
        }
        function refreshStream() {
            var img = document.getElementById('video-stream');
            var src = img.src;
            img.src = '';
            img.src = src + '?t=' + new Date().getTime();
            updateStatus('Odświeżanie...');
        }
        function takeSnapshot() {
            var img = document.getElementById('video-stream');
            var canvas = document.createElement('canvas');
            var ctx = canvas.getContext('2d');
            canvas.width = img.naturalWidth;
            canvas.height = img.naturalHeight;
            ctx.drawImage(img, 0, 0);
            var link = document.createElement('a');
            link.download = 'puck_tracker_snapshot_' + new Date().getTime() + '.jpg';
            link.href = canvas.toDataURL('image/jpeg', 0.9);
            link.click();
        }
        function toggleFullscreen() {
            var img = document.getElementById('video-stream');
            if (img.requestFullscreen) img.requestFullscreen();
            else if (img.webkitRequestFullscreen) img.webkitRequestFullscreen();
            else if (img.msRequestFullscreen) img.msRequestFullscreen();
        }

        // Auto-refresh intervals
        setInterval(fetchRobotStatus, 2000);
        setInterval(fetchDetectionStatus, 1000);

        // Initialize
        window.addEventListener('load', function() {
            fetchRobotStatus();
            fetchDetectionStatus();
            loadCalibrationData();
        });

        // Auto-refresh stream if it fails
        setInterval(function() {
            var img = document.getElementById('video-stream');
            if (img.complete && img.naturalHeight === 0) refreshStream();
        }, 30000);
    </script>
</body>
</html>
)html";
}

// ======= Robot/Calibration endpoints (robot origin) =======
std::string handle_robot_calibration_endpoints(const std::string& request) {
    // POST /calibration/robot_origin — ustawia origin robota z aktualnej pozycji
    if (request.find("POST /calibration/robot_origin") != std::string::npos) {
        bool robot_pos_acquired = false;
        cv::Point3f current_robot_pos(0, 0, 50);

#ifdef USE_ABB
        std::lock_guard<std::mutex> lk(g_robot_mx);
        if (g_robot && g_robot->isConnected()) {
            try {
                RobotPosition pos = g_robot->getCurrentPosition();
                if (pos.valid) {
                    current_robot_pos = cv::Point3f(pos.x, pos.y, pos.z);
                    robot_pos_acquired = true;
                } else {
                    double px, py, pz, q0, qx, qy, qz;
                    if (g_robot->queryCartesianPosition(px, py, pz, q0, qx, qy, qz)) {
                        current_robot_pos = cv::Point3f(px, py, pz);
                        robot_pos_acquired = true;
                    }
                }
            } catch (const std::exception& e) {
                g_robot_diag.last_error = "Calibration error: " + std::string(e.what());
            }
        }
#endif

        if (robot_pos_acquired) {
            g_robot_calibration.robot_origin = current_robot_pos; // [mm]
            g_robot_calibration.is_calibrated = true;
            g_robot_calibration.calibration_time = std::chrono::high_resolution_clock::now();
            return std::string("{\"status\":\"ok\",\"message\":\"Robot origin set from current pose\"}");
        } else {
            return std::string("{\"status\":\"error\",\"message\":\"Robot not connected or pose unavailable\"}");
        }
    }

    // GET /calibration/robot_status
    if (request.find("GET /calibration/robot_status") != std::string::npos) {
        std::ostringstream os;
        os << "{"
           << "\"calibrated\":" << (g_robot_calibration.is_calibrated ? "true" : "false") << ","
           << "\"robot_origin_mm\":{\"x\":" << g_robot_calibration.robot_origin.x
           << ",\"y\":" << g_robot_calibration.robot_origin.y
           << ",\"z\":" << g_robot_calibration.robot_origin.z << "}"
           << "}";
        return os.str();
    }

    // POST /robot/move_to_origin — przemieść do (0,0) z offsetem
    if (request.find("POST /robot/move_to_origin") != std::string::npos) {
        if (!g_robot_calibration.is_calibrated) {
            return "{\"status\":\"error\",\"message\":\"Robot origin not calibrated\"}";
        }
#ifdef USE_ABB
        std::lock_guard<std::mutex> lk(g_robot_mx);
        if (g_robot && g_robot->isConnected()) {
            try {
                float x = g_robot_calibration.robot_origin.x / 1000.0f;
                float y = g_robot_calibration.robot_origin.y / 1000.0f;
                float z = (g_robot_calibration.robot_origin.z + 50.0f) / 1000.0f; // +5 cm

                g_robot->setSpeed(100.0, 50.0);
                g_robot->moveCartesian(x, y, z, 1.0, 0.0, 0.0, 0.0);

                std::ostringstream os;
                os << "{\"status\":\"moving_to_origin\",\"target\":{\"x\":" << x
                   << ",\"y\":" << y << ",\"z\":" << z << "}}";
                return os.str();
            } catch (const std::exception& e) {
                return "{\"status\":\"error\",\"message\":\"Move failed\"}";
            }
        }
#endif
        return "{\"status\":\"error\",\"message\":\"Robot not connected\"}";
    }

    return "{\"status\":\"unknown_endpoint\"}";
}

// ======= HTTP handler =======
void handle_client(int client_socket, cv::VideoCapture& cap) {
    char buffer[8192] = {0};
    recv(client_socket, buffer, sizeof(buffer)-1, 0);
    std::string request(buffer);

    if (request.find("GET / ") == 0) {
        send_http_response(client_socket, "text/html", get_html_page());
    }
    else if (request.find("GET /stream") == 0) {
        send_mjpeg_stream(client_socket, cap);
    }
    else if (request.find("GET /calibration/original") == 0) {
        cv::Mat frame; cap >> frame;
        if (!frame.empty()) send_http_response(client_socket, "image/jpeg", mat_to_jpeg(frame));
        else send_http_response(client_socket, "text/plain", "no frame");
    }
    else if (request.find("GET /calibration/mask") == 0) {
        cv::Mat frame; cap >> frame;
        if (!frame.empty()) {
            cv::Mat mask = generate_mask(frame);
            send_http_response(client_socket, "image/jpeg", mat_to_jpeg(mask));
        } else send_http_response(client_socket, "text/plain", "no frame");
    }
    else if (request.find("GET /calibration/result") == 0) {
        cv::Mat frame; cap >> frame;
        if (!frame.empty()) {
            cv::Mat mask = generate_mask(frame);
            cv::Mat result = generate_result(frame, mask);
            send_http_response(client_socket, "image/jpeg", mat_to_jpeg(result));
        } else send_http_response(client_socket, "text/plain", "no frame");
    }
    else if (request.find("GET /calibration/settings") == 0) {
        std::ostringstream os;
        os << "{"
           << "\"hsv_lower\":[" << g_calibration.hsv_lower[0] << "," << g_calibration.hsv_lower[1] << "," << g_calibration.hsv_lower[2] << "],"
           << "\"hsv_upper\":[" << g_calibration.hsv_upper[0] << "," << g_calibration.hsv_upper[1] << "," << g_calibration.hsv_upper[2] << "],"
           << "\"min_area\":" << g_calibration.min_area << ","
           << "\"max_area\":" << g_calibration.max_area << ","
           << "\"min_circularity\":" << g_calibration.min_circularity << ","
           << "\"use_background_subtraction\":" << (g_calibration.use_background_subtraction ? "true" : "false") << ","
           << "\"table_roi\":{\"x\":" << g_calibration.table_roi.x << ",\"y\":" << g_calibration.table_roi.y
           << ",\"width\":" << g_calibration.table_roi.width << ",\"height\":" << g_calibration.table_roi.height << "}"
           << "}";
        send_http_response(client_socket, "application/json", os.str());
    }
    else if (request.find("GET /detection/status") == 0) {
        send_http_response(client_socket, "application/json", detection_status_json());
    }
    else if (request.find("POST /calibration/hsv") == 0) {
        // (opcjonalnie) sparsować body i ustawić g_calibration
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/area") == 0) {
        // (opcjonalnie) sparsować body i ustawić g_calibration
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/method") == 0) {
        g_calibration.use_background_subtraction = !g_calibration.use_background_subtraction;
        send_http_response(client_socket, "application/json", "{\"status\":\"ok\"}");
    }
    else if (request.find("POST /calibration/save") == 0) {
        std::ofstream file("calibration.json");
        file << detection_status_json(); // szybki dump stanu detekcji (lub zrób osobny JSON parametrów)
        file.close();
        send_http_response(client_socket, "application/json", "{\"status\":\"saved\"}");
    }
    else if (request.find("POST /calibration/load") == 0) {
        std::ifstream file("calibration.json");
        if (file.is_open()) {
            send_http_response(client_socket, "application/json", "{\"status\":\"loaded\"}");
        } else {
            send_http_response(client_socket, "application/json", "{\"status\":\"error\",\"message\":\"file not found\"}");
        }
    }
    // ===== Kalibracja stołu =====
    else if (request.rfind("POST /table/calibrate", 0) == 0) {
        auto p = request.find("\r\n\r\n");
        if (p == std::string::npos) {
            send_http_response(client_socket,"application/json","{\"status\":\"error\",\"msg\":\"no body\"}");
        } else {
            std::string body = request.substr(p+4);
            auto nums = parse_numbers(body);
            // oczekujemy: x1 y1 x2 y2 x3 y3 x4 y4 width height origin clockwise
            if (nums.size() < 12) {
                send_http_response(client_socket,"application/json","{\"status\":\"error\",\"msg\":\"need 12 numbers\"}");
            } else {
                g_table_calib.img_pts.clear();
                g_table_calib.img_pts.emplace_back((float)nums[0], (float)nums[1]);
                g_table_calib.img_pts.emplace_back((float)nums[2], (float)nums[3]);
                g_table_calib.img_pts.emplace_back((float)nums[4], (float)nums[5]);
                g_table_calib.img_pts.emplace_back((float)nums[6], (float)nums[7]);
                g_table_calib.width_mm     = (float)nums[8];
                g_table_calib.height_mm    = (float)nums[9];
                g_table_calib.origin_index = std::clamp<int>((int)std::lround(nums[10]),0,3);
                g_table_calib.clockwise    = (nums[11] >= 0.5);
                g_table_calib.computeHomography();

                send_http_response(client_socket,"application/json",
                    std::string("{\"status\":\"ok\",\"ready\":") + (g_table_calib.ready?"true":"false") + "}");
            }
        }
    }
    else if (request.rfind("GET /table/status", 0) == 0) {
        std::ostringstream os;
        os << "{"
           << "\"ready\":" << (g_table_calib.ready?"true":"false") << ","
           << "\"width_mm\":" << g_table_calib.width_mm << ","
           << "\"height_mm\":" << g_table_calib.height_mm << ","
           << "\"origin_index\":" << g_table_calib.origin_index << ","
           << "\"clockwise\":" << (g_table_calib.clockwise?"true":"false")
           << "}";
        send_http_response(client_socket,"application/json", os.str());
    }
    else if (request.rfind("POST /table/reset", 0) == 0) {
        g_table_calib = TableCalibration{};
        send_http_response(client_socket,"application/json","{\"status\":\"reset\"}");
    }
    // ===== Robot control endpoints =====
    else if (request.rfind("GET /robot/status", 0) == 0) {
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.rfind("POST /robot/connect", 0) == 0) {
        // parse query ?ip=...&port=...
        std::string ip = g_robot_diag.ip;
        uint16_t port = g_robot_diag.port;
        auto qpos = request.find('?');
        if (qpos != std::string::npos) {
            auto qs = request.substr(qpos+1, request.find(' ') - (qpos+1));
            std::map<std::string,std::string> kv;
            std::stringstream ss(qs);
            std::string token;
            while (std::getline(ss, token, '&')) {
                auto eq = token.find('=');
                if (eq!=std::string::npos) kv[token.substr(0,eq)] = token.substr(eq+1);
            }
            if (kv.count("ip"))   ip   = kv["ip"];
            if (kv.count("port")) port = static_cast<uint16_t>(std::stoi(kv["port"]));
        }

        bool ok = false;
        std::string err;

#ifdef USE_ABB
        {
            std::lock_guard<std::mutex> lk(g_robot_mx);
            if (g_robot) { try { g_robot->disconnect(); } catch(...) {} delete g_robot; g_robot = nullptr; }
            g_robot_diag.ip = ip;
            g_robot_diag.port = port;
            g_robot_diag.last_error.clear();
        }
        try {
            // AbbRobotController(ip, motion_port, logger_port)
            auto* r = new AbbRobotController(ip, port, static_cast<int>(port) + 1);
            

            r->setPositionCallback([](const RobotPosition& pos) {
                if (!pos.valid) return;
                // mm -> m do strategii
                cv::Point2f rp(pos.x / 1000.0f, pos.y / 1000.0f);
                // lekki filtr/ochrona
                {
                    std::lock_guard<std::mutex> lk(g_detection_mutex);
                    g_strategy.robot_position = rp;
                }
            });
            if (r->connect()) {
                std::lock_guard<std::mutex> lk(g_robot_mx);
                g_robot = r; ok = true;
            } else {
                delete r; err = "connect() returned false";
            }
        } catch (const std::exception& e) { err = e.what(); }

        if (!ok) {
            std::lock_guard<std::mutex> lk(g_robot_mx);
            g_robot_diag.last_error = err.empty() ? "Unknown error" : err;
        }
#else
        {
            std::lock_guard<std::mutex> lk(g_robot_mx);
            g_robot_diag.last_error = "ABB disabled at compile time (USE_ABB=OFF)";
        }
#endif
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.rfind("POST /robot/disconnect", 0) == 0) {
#ifdef USE_ABB
        {
            std::lock_guard<std::mutex> lk(g_robot_mx);
            if (g_robot) { try { g_robot->disconnect(); } catch(...) {} delete g_robot; g_robot = nullptr; }
            g_robot_diag.last_error = "Disconnected by user";
        }
#else
        {
            std::lock_guard<std::mutex> lk(g_robot_mx);
            g_robot_diag.last_error = "ABB disabled at compile time (USE_ABB=OFF)";
        }
#endif
        send_http_response(client_socket, "application/json", robot_status_json());
    }
    else if (request.rfind("POST /robot/strategy", 0) == 0) {
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body = request.substr(body_start + 4);
            if (body.find("defensive") != std::string::npos)      g_strategy.current_mode = GameStrategy::DEFENSIVE;
            else if (body.find("offensive") != std::string::npos) g_strategy.current_mode = GameStrategy::OFFENSIVE;
            else if (body.find("intercept") != std::string::npos) g_strategy.current_mode = GameStrategy::INTERCEPT;
        }
        send_http_response(client_socket, "application/json", "{\"status\":\"strategy_updated\"}");
    }
    else if (request.rfind("POST /robot/emergency_stop", 0) == 0) {
#ifdef USE_ABB
        std::lock_guard<std::mutex> lk(g_robot_mx);
        if (g_robot) {
            try {
                g_robot->disconnect();
                delete g_robot; g_robot = nullptr;
                g_robot_diag.last_error = "Emergency stop executed (connection closed)";
            } catch (const std::exception& e) {
                g_robot_diag.last_error = std::string("Emergency stop failed: ") + e.what();
            }
        }
#else
        {
            std::lock_guard<std::mutex> lk(g_robot_mx);
            g_robot_diag.last_error = "Emergency stop (SIMULATION)";
        }
#endif
        send_http_response(client_socket, "application/json", "{\"status\":\"emergency_stop\"}");
    }
    else if (request.rfind("POST /robot/manual_move", 0) == 0) {
        // pobierz (opcjonalnie) x,y,z,speed z body (metry)
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
        RobotCommand cmd{x,y,z,0.0f,speed,false};
        sendRobotCommand(cmd);
        send_http_response(client_socket, "application/json", "{\"status\":\"manual_move_sent\"}");
    }
    else if (request.rfind("POST /robot/strike", 0) == 0) {
        RobotCommand cmd = calculateRobotCommandCalibrated(g_current_puck, g_predictor);
        cmd.execute_strike = true; cmd.speed = 1.0f;
        sendRobotCommand(cmd);
        send_http_response(client_socket, "application/json", "{\"status\":\"strike_executed\"}");
    }
    // ===== Robot-origin / robot move-to-origin =====
    else if (request.find("POST /calibration/robot_origin") != std::string::npos ||
             request.find("GET /calibration/robot_status") != std::string::npos ||
             request.find("POST /robot/move_to_origin") != std::string::npos) {
        std::string response = handle_robot_calibration_endpoints(request);
        send_http_response(client_socket, "application/json", response);
    }
    else {
        std::string not_found = "<h1>404 Not Found</h1>";
        send_http_response(client_socket, "text/html", not_found);
    }

    close(client_socket);
}

int main() {
    std::cout << "=== PUCK TRACKER + ROBOT CONTROL SYSTEM ===" << std::endl;

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "❌ Nie można otworzyć kamery!" << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    g_bg_subtractor = cv::createBackgroundSubtractorMOG2(500, 16, false);

    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        std::cerr << "❌ Nie można utworzyć socket!" << std::endl;
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
    if (listen(server_socket, 5) < 0) {
        std::cerr << "❌ Listen failed!" << std::endl;
        return -1;
    }

    std::cout << "🌐 Serwer działa na porcie 8080 (http://<IP_RPI>:8080)" << std::endl;
    std::cout << "🎯 Detekcja krążka: AKTYWNA" << std::endl;
#ifdef USE_ABB
    std::cout << "🤖 Kontrola robota: WŁĄCZONA (USE_ABB=ON)" << std::endl;
#else
    std::cout << "🤖 Kontrola robota: SYMULACJA (USE_ABB=OFF)" << std::endl;
#endif

    while (running) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
        if (client_socket < 0) continue;
        std::thread client_thread(handle_client, client_socket, std::ref(cap));
        client_thread.detach();
    }

    // Cleanup
    close(server_socket);
    cap.release();

#ifdef USE_ABB
    {
        std::lock_guard<std::mutex> lk(g_robot_mx);
        if (g_robot) {
            try { g_robot->disconnect(); } catch(...) {}
            delete g_robot; g_robot = nullptr;
        }
    }
#endif

    std::cout << "🛑 Serwer zakończył działanie" << std::endl;
    return 0;
}
