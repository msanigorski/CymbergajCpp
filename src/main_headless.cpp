//main_headless.cpp
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <getopt.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

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
    int fps = 60;  // Realistyczne dla RPi3
    std::string backend = "v4l2";  // Używamy V4L2 zamiast GStreamer
    std::string method = "color";
    std::string config_file = "config/config.yaml";
    int queue_size = 5;  // Mniejsza kolejka
    bool enable_profiling = true;
    bool save_output = false;
    std::string output_file = "tracking_data.csv";

    // Dodane brakujące opcje:
    std::string pipeline = "";       // dla -p / --pipeline
    bool show_window = true;         // dla -n / --no-window
    bool enable_stream = false;      // dla -s / --stream
    std::string stream_address = ""; // adres strumienia
};

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
        {"help", no_argument, 0, '?'},
        {0, 0, 0, 0}
    };
    
    int opt;
    int option_index = 0;
    
    while ((opt = getopt_long(argc, argv, "w:h:f:b:p:m:c:ns:q:?", 
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
            case '?':
                std::cout << "Usage: " << argv[0] << " [options]\n"
                         << "Options:\n"
                         << "  -w, --width WIDTH       Frame width (default: 640)\n"
                         << "  -h, --height HEIGHT     Frame height (default: 480)\n"
                         << "  -f, --fps FPS           Target FPS (default: 90)\n"
                         << "  -b, --backend BACKEND   Capture backend: gstreamer|libcamera (default: gstreamer)\n"
                         << "  -p, --pipeline PIPELINE Custom GStreamer pipeline\n"
                         << "  -m, --method METHOD     Detection method: color|bgsub (default: color)\n"
                         << "  -c, --config FILE       Config file path (default: config/config.yaml)\n"
                         << "  -n, --no-window         Disable display window\n"
                         << "  -s, --stream ADDRESS    Enable streaming to ADDRESS\n"
                         << "  -q, --queue-size SIZE   Frame queue size (default: 10)\n"
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
                   std::ofstream* output_file = nullptr) {
    
    FrameData frame_data;
    
    while (running) {
        if (input_queue->pop(frame_data)) {
            auto start = high_resolution_clock::now();
            
            cv::Point2f puck_pos;
            bool detected = detector->detect(frame_data.frame, puck_pos);
            
            if (detected) {
                tracker->update(puck_pos, frame_data.timestamp);
            } else {
                tracker->predictOnly(frame_data.timestamp);
            }
            
            // Zapisz dane do pliku CSV
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
            
            // Wyświetl status w konsoli
            if (frame_data.frame_id % 30 == 0) {  // Co 30 klatek
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
    
    // Otwórz plik wyjściowy
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
    
    std::cout << "Starting puck tracker (headless mode)..." << std::endl;
    std::cout << "Resolution: " << cfg.width << "x" << cfg.height 
              << " @ " << cfg.fps << " FPS" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    // Start threads
    std::thread capture_t(capture_thread, capture.get(), &capture_queue, 
                         std::ref(g_running), stats.get());
    
    std::thread process_t(process_thread, &capture_queue,
                         detector.get(), tracker.get(), std::ref(g_running),
                         stats.get(), cfg.save_output ? &output_file : nullptr);
    
    // Performance monitoring loop
    auto last_stats_update = high_resolution_clock::now();
    int frames_processed = 0;
    
    while (g_running) {
        std::this_thread::sleep_for(seconds(1));
        
        auto now = high_resolution_clock::now();
        auto elapsed = duration_cast<seconds>(now - last_stats_update).count();
        
        if (elapsed >= 1 && cfg.enable_profiling) {
            float fps = frames_processed / static_cast<float>(elapsed);
            
            std::cout << "\n[STATS] FPS: " << std::fixed << std::setprecision(1) << fps
                     << " | Latency: " << stats->getLatency() << "ms"
                     << " | Drops: " << stats->dropped_frames
                     << " | Cap: " << stats->capture_time << "μs"
                     << " | Proc: " << stats->process_time << "μs" << std::endl;
            
            frames_processed = 0;
            last_stats_update = now;
        }
    }
    
    // Cleanup
    capture_t.join();
    process_t.join();
    
    if (output_file.is_open()) {
        output_file.close();
    }
    
    std::cout << "\nShutdown complete." << std::endl;
    return 0;
}