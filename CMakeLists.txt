cmake_minimum_required(VERSION 3.16)
project(puck_tracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_HEADLESS "Build without GUI support" ON)
option(USE_ABB "Enable ABB integration if abb_comm is present" ON)
option(BUILD_TESTS "Build tests" OFF)

# --- OpenCV
if(BUILD_HEADLESS)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)
else()
    find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui video videoio)
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- Sources
set(SOURCES
    src/capture.cpp
    src/detector.cpp
    src/tracker.cpp
    src/utils.cpp
)

if(BUILD_HEADLESS)
    # HTTP-capable headless main
    list(APPEND SOURCES src/main_headless_with_http.cpp)
else()
    list(APPEND SOURCES src/main.cpp src/overlay.cpp)
endif()

# --- Main target
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
        opencv_video          # explicitly link video just in case
        Threads::Threads
        yaml-cpp
        atomic
)

# Define USE_ABB for the app (after target exists)
if(USE_ABB)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ABB=1)
endif()

# --- RPi legacy camera (if present)
if(EXISTS /opt/vc/include)
    target_include_directories(${PROJECT_NAME} PRIVATE /opt/vc/include)
    target_link_directories(${PROJECT_NAME} PRIVATE /opt/vc/lib)
    target_link_libraries(${PROJECT_NAME} PRIVATE mmal_core mmal_util mmal_vc_client bcm_host)
endif()

# =========================
#  ABB (optional)
# =========================
if(USE_ABB)
    add_library(abb_robot
        src/robot/abb/AbbController.cpp
        src/robot/abb/RobotStrikePlanner.cpp
        external/abb_comm.cpp
    )
    target_include_directories(abb_robot PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/external
        ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_definitions(abb_robot PUBLIC USE_ABB=1)
    target_link_libraries(abb_robot PUBLIC ${OpenCV_LIBS})
    target_link_libraries(${PROJECT_NAME} PRIVATE abb_robot)
else()
    message(STATUS "ABB: USE_ABB=OFF — building without ABB support")
endif()
