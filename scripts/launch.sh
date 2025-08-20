#!/bin/bash

# Puck Tracker Launch Script for Raspberry Pi
# Optimized for 90 FPS operation

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Puck Tracker Launch Script ===${NC}"

# Check if running on Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model)
    echo -e "${GREEN}Detected: $MODEL${NC}"
else
    echo -e "${YELLOW}Warning: Not running on Raspberry Pi${NC}"
fi

# Function to check if running as root
check_root() {
    if [ "$EUID" -eq 0 ]; then
        echo -e "${GREEN}Running as root - thread priorities will be set${NC}"
        return 0
    else
        echo -e "${YELLOW}Not running as root - thread priorities disabled${NC}"
        echo -e "${YELLOW}For best performance, run with: sudo $0${NC}"
        return 1
    fi
}

# Function to set CPU governor for maximum performance
set_performance_mode() {
    if check_root; then
        echo -e "${GREEN}Setting CPU to performance mode...${NC}"
        for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo performance > $cpu 2>/dev/null
        done
        
        # Disable CPU frequency scaling
        echo -e "${GREEN}Disabling CPU frequency scaling...${NC}"
        for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq; do
            cat ${cpu/min/max} > $cpu 2>/dev/null
        done
    fi
}

# Function to optimize GPU memory split
check_gpu_memory() {
    GPU_MEM=$(vcgencmd get_mem gpu | cut -d= -f2 | cut -d'M' -f1)
    if [ "$GPU_MEM" -lt 128 ]; then
        echo -e "${YELLOW}Warning: GPU memory is only ${GPU_MEM}MB${NC}"
        echo -e "${YELLOW}For better performance, add 'gpu_mem=128' to /boot/config.txt${NC}"
    else
        echo -e "${GREEN}GPU memory: ${GPU_MEM}MB${NC}"
    fi
}

# Function to disable unnecessary services
optimize_system() {
    if check_root; then
        echo -e "${GREEN}Optimizing system for real-time performance...${NC}"
        
        # Stop unnecessary services temporarily
        systemctl stop bluetooth 2>/dev/null
        systemctl stop avahi-daemon 2>/dev/null
        
        # Disable WiFi power management
        iwconfig wlan0 power off 2>/dev/null
        
        # Set swappiness to 0 for less swap usage
        echo 0 > /proc/sys/vm/swappiness
        
        # Increase kernel timer frequency (if possible)
        echo 1000 > /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null
    fi
}

# Parse command line arguments
BACKEND="gstreamer"
METHOD="color"
FPS=90
WIDTH=640
HEIGHT=480
CONFIG="config/config.yaml"
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --libcamera)
            BACKEND="libcamera"
            shift
            ;;
        --bgsub)
            METHOD="bgsub"
            shift
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        --res)
            WIDTH="${2%x*}"
            HEIGHT="${2#*x}"
            shift 2
            ;;
        --config)
            CONFIG="$2"
            shift 2
            ;;
        --debug)
            EXTRA_ARGS="$EXTRA_ARGS --log-level debug"
            shift
            ;;
        --no-window)
            EXTRA_ARGS="$EXTRA_ARGS --no-window"
            shift
            ;;
        --stream)
            EXTRA_ARGS="$EXTRA_ARGS --stream $2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --libcamera     Use libcamera backend instead of GStreamer"
            echo "  --bgsub         Use background subtraction instead of color detection"
            echo "  --fps N         Set target FPS (default: 90)"
            echo "  --res WxH       Set resolution (default: 640x480)"
            echo "  --config FILE   Config file path (default: config/config.yaml)"
            echo "  --debug         Enable debug logging"
            echo "  --no-window     Disable display window"
            echo "  --stream ADDR   Enable streaming to address"
            echo "  --help          Show this help"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# System optimization
echo -e "${GREEN}Performing system optimization...${NC}"
set_performance_mode
check_gpu_memory
optimize_system

# Build the command
CMD="./puck_tracker"
CMD="$CMD --backend $BACKEND"
CMD="$CMD --width $WIDTH --height $HEIGHT --fps $FPS"
CMD="$CMD --method $METHOD"
CMD="$CMD --config $CONFIG"
CMD="$CMD $EXTRA_ARGS"

# Special handling for GStreamer pipeline
if [ "$BACKEND" = "gstreamer" ]; then
    # Optimized pipeline for 90 FPS
    PIPELINE="libcamerasrc ! video/x-raw,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1,format=BGR ! videoconvert ! appsink drop=true max-buffers=2 sync=false"
    CMD="$CMD --pipeline \"$PIPELINE\""
fi

# Check if binary exists
if [ ! -f "./puck_tracker" ]; then
    echo -e "${RED}Error: puck_tracker binary not found!${NC}"
    echo -e "${YELLOW}Please build the project first:${NC}"
    echo "  mkdir build && cd build"
    echo "  cmake .. && make -j4"
    exit 1
fi

# Launch the application
echo -e "${GREEN}Launching puck tracker...${NC}"
echo -e "${YELLOW}Command: $CMD${NC}"
echo -e "${GREEN}Press Ctrl+C to stop${NC}"
echo ""

# Set process priority if root
if check_root; then
    # Run with real-time priority
    nice -n -20 ionice -c 1 -n 0 bash -c "$CMD"
else
    eval $CMD
fi

# Cleanup on exit
echo -e "\n${GREEN}Cleaning up...${NC}"
if check_root; then
    # Restore CPU governor
    for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        echo ondemand > $cpu 2>/dev/null
    done
    
    # Restore services
    systemctl start bluetooth 2>/dev/null
    systemctl start avahi-daemon 2>/dev/null
fi

echo -e "${GREEN}Done!${NC}"