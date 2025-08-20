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