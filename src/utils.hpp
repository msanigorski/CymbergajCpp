// utils.hpp
#pragma once

#include <chrono>
#include <atomic>
#include <string>

class PerfStats {
public:
    std::atomic<float> fps{0};
    std::atomic<int64_t> capture_time{0};  // microseconds
    std::atomic<int64_t> process_time{0};  // microseconds
    std::atomic<int64_t> render_time{0};   // microseconds
    std::atomic<int> dropped_frames{0};
    
    float getLatency() const {
        return (capture_time + process_time + render_time) / 1000.0f;  // Convert to ms
    }
};

class Timer {
public:
    Timer() : start_(std::chrono::high_resolution_clock::now()) {}
    
    void reset() {
        start_ = std::chrono::high_resolution_clock::now();
    }
    
    double elapsed_ms() const {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(end - start_).count();
    }
    
    int64_t elapsed_us() const {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
    }
    
private:
    std::chrono::high_resolution_clock::time_point start_;
};

// Simple logger
class Logger {
public:
    enum Level {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        ERROR = 3
    };
    
    static void log(Level level, const std::string& message);
    static void setLevel(Level level) { min_level_ = level; }
    
private:
    static Level min_level_;
};

// utils.cpp
#include "utils.hpp"
#include <iostream>
#include <iomanip>
#include <ctime>

Logger::Level Logger::min_level_ = Logger::INFO;

void Logger::log(Level level, const std::string& message) {
    if (level < min_level_) return;
    
    const char* level_str[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    std::cout << "." << std::setfill('0') << std::setw(3) << ms.count();
    std::cout << "] [" << level_str[level] << "] " << message << std::endl;
}