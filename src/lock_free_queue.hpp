#pragma once

#include <atomic>
#include <vector>
#include <opencv2/core.hpp>

struct FrameData {
    cv::Mat frame;
    uint64_t timestamp;
    uint64_t frame_id;
};

struct ProcessedFrame {
    cv::Mat frame;
    uint64_t timestamp;
    uint64_t frame_id;
    cv::Point2f puck_position;
    cv::Point2f puck_velocity;
    std::vector<cv::Point2f> trajectory;
    bool detected;
};

template<typename T>
class LockFreeQueue {
private:
    std::vector<T> buffer_;
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
    const size_t capacity_;
    
public:
    explicit LockFreeQueue(size_t capacity) 
        : buffer_(capacity + 1), capacity_(capacity + 1) {}
    
    bool push(const T& item) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % capacity_;
        
        if (next_tail == head_.load(std::memory_order_acquire)) {
            // Queue is full
            return false;
        }
        
        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }
    
    bool pop(T& item) {
        size_t current_head = head_.load(std::memory_order_relaxed);
        
        if (current_head == tail_.load(std::memory_order_acquire)) {
            // Queue is empty
            return false;
        }
        
        item = std::move(buffer_[current_head]);
        head_.store((current_head + 1) % capacity_, std::memory_order_release);
        return true;
    }
    
    bool empty() const {
        return head_.load(std::memory_order_acquire) == 
               tail_.load(std::memory_order_acquire);
    }
    
    size_t size() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (t >= h) ? (t - h) : (capacity_ - h + t);
    }
};