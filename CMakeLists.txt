cmake_minimum_required(VERSION 3.16)
project(puck_tracker VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(USE_ABB "Enable ABB robot support" OFF)
option(BUILD_TESTS "Build test applications" ON)

# Find packages
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src)  # Add src directory to include path

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -latomic")
endif()
# ABB Robot support
if(USE_ABB)
    message(STATUS "ABB robot support enabled")
    add_definitions(-DUSE_ABB)
    
    # Include ABB headers and sources - using local files
    include_directories(src/robot/abb)
    
    # Check if the required ABB files exist first
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/robot/abb/abb_comm.cpp")
        message(STATUS "Found ABB communication implementation: src/robot/abb/abb_comm.cpp")
    else()
        message(FATAL_ERROR "ABB communication file not found: src/robot/abb/abb_comm.cpp")
    endif()
    
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/robot/abb/abb_comm.h")
        message(STATUS "Found ABB communication header: src/robot/abb/abb_comm.h")
    else()
        message(FATAL_ERROR "ABB communication header not found: src/robot/abb/abb_comm.h")
    endif()
    
    # Add ABB controller sources
    set(ABB_SOURCES
        src/robot/abb/abb_comm.cpp
    )
    
    # Add AbbRobotController.cpp if it exists (new enhanced controller)
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/robot/abb/AbbRobotController.cpp")
        message(STATUS "Found enhanced ABB robot controller: src/robot/abb/AbbRobotController.cpp")
        list(APPEND ABB_SOURCES src/robot/abb/AbbRobotController.cpp)
    else()
        message(STATUS "AbbRobotController.cpp not found - you may need to create it")
    endif()
    
    # Add legacy AbbController.cpp if it exists
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/robot/abb/AbbController.cpp")
        message(STATUS "Found legacy ABB controller implementation: src/robot/abb/AbbController.cpp")
        list(APPEND ABB_SOURCES src/robot/abb/AbbController.cpp)
    else()
        message(STATUS "Legacy AbbController.cpp not found - using only abb_comm and AbbRobotController")
    endif()
    
else()
    message(STATUS "ABB robot support disabled - using simulation mode")
    set(ABB_SOURCES "")
endif()

# Common sources - dodaj brakujące pliki
set(COMMON_SOURCES
    src/capture.cpp
    src/detector.cpp  
    src/tracker.cpp
    src/utils.cpp
)

# Add BUILD_HEADLESS option at the top with other options
option(BUILD_HEADLESS "Build headless version with HTTP calibration" ON)

# Main puck tracker executable
if(BUILD_HEADLESS)
    message(STATUS "Building headless version with HTTP interface")
    add_executable(puck_tracker
        src/main_headless_with_http.cpp
        ${ABB_SOURCES}
        ${COMMON_SOURCES}
    )
else()
    message(STATUS "Building full GUI version")
    add_executable(puck_tracker
        src/http_puck_tracker.cpp
        ${ABB_SOURCES}
        ${COMMON_SOURCES}
    )
endif()

target_link_libraries(puck_tracker
    ${OpenCV_LIBS}
    Threads::Threads
    yaml-cpp
    atomic
)

# Robot test module executable
add_executable(robot_test_http
    src/robot_test_http.cpp
    ${ABB_SOURCES}
    ${COMMON_SOURCES}
)

target_link_libraries(robot_test_http
    ${OpenCV_LIBS}
    Threads::Threads
    yaml-cpp
    atomic
)

find_package(yaml-cpp REQUIRED)

# I w target_link_libraries:
target_link_libraries(puck_tracker
    ${OpenCV_LIBS}
    Threads::Threads
    yaml-cpp
)

# Test applications (optional)
if(BUILD_TESTS)
    # Enhanced robot test with logger support
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/robot/abb/enhanced_robot_test.cpp")
        add_executable(enhanced_robot_test
            src/robot/abb/enhanced_robot_test.cpp
            ${ABB_SOURCES}
        )
        
        target_link_libraries(enhanced_robot_test
            Threads::Threads
        )
        
        message(STATUS "Enhanced robot test (with logger) will be built")
    endif()
    
    # Simple robot test - console application (legacy)
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/robot/abb/simple_robot_test.cpp")
        add_executable(simple_robot_test
            src/robot/abb/simple_robot_test.cpp
            ${ABB_SOURCES}
        )
        
        target_link_libraries(simple_robot_test
            Threads::Threads
        )
        
        message(STATUS "Simple robot test will be built")
    endif()
    
    # Camera test - only build if file exists
    if(EXISTS "${CMAKE_SOURCE_DIR}/tests/camera_test.cpp")
        add_executable(camera_test
            tests/camera_test.cpp
        )
        
        target_link_libraries(camera_test
            ${OpenCV_LIBS}
        )
    else()
        message(STATUS "camera_test.cpp not found - skipping camera test build")
    endif()
endif()

# Installation
install(TARGETS puck_tracker robot_test_http
    RUNTIME DESTINATION bin
)

# Optional: Install configuration files
install(FILES
    config/calibration.json
    config/robot_config.json
    DESTINATION share/puck_tracker
    OPTIONAL
)

# Print configuration summary
message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "  ABB Support: ${USE_ABB}")
message(STATUS "  Build Tests: ${BUILD_TESTS}")
message(STATUS "")
message(STATUS "Targets to build:")
message(STATUS "  puck_tracker - Main application with puck detection and robot control")
message(STATUS "  robot_test_http - Robot testing module with web interface")
if(BUILD_TESTS)
    message(STATUS "  simple_robot_test - Basic robot movement test")
    message(STATUS "  camera_test - Camera functionality test")
endif()
message(STATUS "")

# Build instructions
message(STATUS "Build Instructions:")
message(STATUS "  mkdir build && cd build")
if(USE_ABB)
    message(STATUS "  cmake .. -DUSE_ABB=ON -DCMAKE_BUILD_TYPE=Release")
else()
    message(STATUS "  cmake .. -DUSE_ABB=OFF -DCMAKE_BUILD_TYPE=Release")
endif()
message(STATUS "  make -j$(nproc)")
message(STATUS "")
message(STATUS "Run Instructions:")
message(STATUS "  ./puck_tracker        # Main application")
message(STATUS "  ./robot_test_http     # Robot testing interface")
message(STATUS "  Then open: http://<IP>:8080")
message(STATUS "")