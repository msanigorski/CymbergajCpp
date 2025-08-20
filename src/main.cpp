#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <getopt.h>
#include <yaml-cpp/yaml.h>

#include "capture.hpp"
#include "detector.hpp"
#include "tracker.hpp"
#include "overlay.hpp"
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
    // Camera settings
    int width = 640;
    int height = 480;
    int fps = 90;
    std::string backend = "gstreamer";
    std::string pipeline = "";
    
    // Detection settings
    std::string method = "color";
    
    // Config file
    std::string config_file = "config/config.yaml";
    
    // Output settings
    bool show_window = true;
    bool enable_stream = false;
    std::string stream_address = "udp://127.0.0.1:5000";
    
    // Performance settings
    int queue_size = 10;
    bool enable_profiling = true;
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
            data.frame = frame.clone();  // Pre-allocated in capture
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
                   LockFreeQueue<ProcessedFrame>* output_queue,
                   Detector* detector, KalmanTracker* tracker,
                   std::atomic<bool>& running, PerfStats* stats,
                   std::atomic<int>& detection_method) {
    
    FrameData frame_data;
    
    while (running) {
        if (input_queue->pop(frame_data)) {
            auto start = high_resolution_clock::now();
            
            ProcessedFrame result;
            result.frame = frame_data.frame;
            result.timestamp = frame_data.timestamp;
            result.frame_id = frame_data.frame_id;
            
            // Detection
            detector->setMethod(static_cast<DetectionMethod>(detection_method.load()));
            cv::Point2f puck_pos;
            bool detected = detector->detect(frame_data.frame, puck_pos);
            
            // Tracking & Prediction
            if (detected) {
                tracker->update(puck_pos, frame_data.timestamp);
            } else {
                tracker->predictOnly(frame_data.timestamp);
            }
            
            result.puck_position = tracker->getPosition();
            result.puck_velocity = tracker->getVelocity();
            result.trajectory = tracker->predictTrajectory(1.0);  // 1 second horizon
            result.detected = detected;
            
            output_queue->push(result);
            
            auto end = high_resolution_clock::now();
            stats->process_time = duration_cast<microseconds>(end - start).count();
        } else {
            std::this_thread::sleep_for(microseconds(100));
        }
    }
}

void render_thread(LockFreeQueue<ProcessedFrame>* queue,
                  Overlay* overlay, bool show_window,
                  std::atomic<bool>& running, PerfStats* stats,
                  std::atomic<int>& detection_method) {
    
    ProcessedFrame frame;
    auto last_fps_update = high_resolution_clock::now();
    int frame_count = 0;
    float current_fps = 0;
    
    while (running) {
        if (queue->pop(frame)) {
            auto start = high_resolution_clock::now();
            
            // Draw overlay
            overlay->draw(frame.frame, frame.puck_position, frame.puck_velocity,
                         frame.trajectory, frame.detected);
            
            // Draw stats
            overlay->drawStats(frame.frame, current_fps, stats->getLatency(),
                              stats->dropped_frames, detection_method);
            
            // Display
            if (show_window) {
                cv::imshow("Puck Tracker", frame.frame);
                
                int key = cv::waitKey(1);
                if (key == 'q' || key == 27) {  // q or ESC
                    running = false;
                } else if (key == 'c') {  // Toggle detection method
                    detection_method = (detection_method + 1) % 2;
                } else if (key == 'p') {  // Pause
                    cv::waitKey(0);
                }
            }
            
            // Calculate FPS
            frame_count++;
            auto now = high_resolution_clock::now();
            auto elapsed = duration_cast<milliseconds>(now - last_fps_update).count();
            if (elapsed >= 1000) {
                current_fps = frame_count * 1000.0f / elapsed;
                stats->fps = current_fps;
                frame_count = 0;
                last_fps_update = now;
            }
            
            auto end = high_resolution_clock::now();
            stats->render_time = duration_cast<microseconds>(end - start).count();
        } else {
            std::this_thread::sleep_for(microseconds(100));
        }
    }
}

int main(int argc, char* argv[]) {
    // Parse arguments
    Config cfg = parse_args(argc, argv);
    
    // Signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // Load YAML config
    YAML::Node yaml_config;
    try {
        yaml_config = YAML::LoadFile(cfg.config_file);
    } catch (const std::exception& e) {
        std::cerr << "Warning: Could not load config file: " << e.what() << std::endl;
        std::cerr << "Using default values." << std::endl;
    }
    
    // Initialize components
    auto capture = std::make_unique<FrameCapture>(cfg.backend, cfg.width, 
                                                  cfg.height, cfg.fps);
    if (cfg.backend == "gstreamer" && !cfg.pipeline.empty()) {
        capture->setPipeline(cfg.pipeline);
    }
    
    if (!capture->initialize()) {
        std::cerr << "Failed to initialize capture device!" << std::endl;
        return -1;
    }
    
    auto detector = std::make_unique<Detector>(yaml_config);
    auto tracker = std::make_unique<KalmanTracker>(yaml_config);
    auto overlay = std::make_unique<Overlay>(yaml_config);
    auto stats = std::make_shared<PerfStats>();
    
    // Create queues
    LockFreeQueue<FrameData> capture_queue(cfg.queue_size);
    LockFreeQueue<ProcessedFrame> render_queue(cfg.queue_size);
    
    // Detection method control
    std::atomic<int> detection_method{cfg.method == "color" ? 0 : 1};
    
    std::cout << "Starting puck tracker..." << std::endl;
    std::cout << "Resolution: " << cfg.width << "x" << cfg.height << " @ " << cfg.fps << " FPS" << std::endl;
    std::cout << "Backend: " << cfg.backend << std::endl;
    std::cout << "Detection: " << cfg.method << std::endl;
    std::cout << "Press 'q' to quit, 'c' to change detection method, 'p' to pause" << std::endl;
    
    // Start threads
    std::thread capture_t(capture_thread, capture.get(), &capture_queue, 
                         std::ref(g_running), stats.get());
    
    std::thread process_t(process_thread, &capture_queue, &render_queue,
                         detector.get(), tracker.get(), std::ref(g_running),
                         stats.get(), std::ref(detection_method));
    
    std::thread render_t(render_thread, &render_queue, overlay.get(),
                        cfg.show_window, std::ref(g_running), stats.get(),
                        std::ref(detection_method));
    
    // Set thread priorities (requires root on Linux)
    #ifdef __linux__
    try {
        pthread_t capture_handle = capture_t.native_handle();
        pthread_t process_handle = process_t.native_handle();
        
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
        pthread_setschedparam(capture_handle, SCHED_FIFO, &param);
        
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        pthread_setschedparam(process_handle, SCHED_FIFO, &param);
    } catch (...) {
        std::cerr << "Warning: Could not set thread priorities (need root)" << std::endl;
    }
    #endif
    
    // Performance monitoring loop
    while (g_running) {
        std::this_thread::sleep_for(seconds(1));
        
        if (cfg.enable_profiling && stats->fps > 0) {
            std::cout << "\rFPS: " << std::fixed << std::setprecision(1) << stats->fps
                     << " | Latency: " << stats->getLatency() << "ms"
                     << " | Drops: " << stats->dropped_frames
                     << " | Cap: " << stats->capture_time << "μs"
                     << " | Proc: " << stats->process_time << "μs"
                     << " | Rend: " << stats->render_time << "μs    " << std::flush;
        }
    }
    
    // Cleanup
    capture_t.join();
    process_t.join();
    render_t.join();
    
    cv::destroyAllWindows();
    
    std::cout << "\nShutdown complete." << std::endl;
    return 0;
}