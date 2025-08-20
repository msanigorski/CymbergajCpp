cmake_minimum_required(VERSION 3.16)
project(puck_tracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_HEADLESS "Build without GUI support" ON)

# OpenCV
if(BUILD_HEADLESS)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
else()
    find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui video videoio)
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Źródła
set(SOURCES
    src/capture.cpp
    src/detector.cpp
    src/tracker.cpp
    src/utils.cpp
)

if(BUILD_HEADLESS)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)
else()
    find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui video videoio)
endif()

if(BUILD_HEADLESS)
    list(APPEND SOURCES src/main_headless.cpp)
else()
    list(APPEND SOURCES src/main.cpp src/overlay.cpp)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Threads::Threads
    yaml-cpp
    atomic
)

# Dla RPi camera
if(EXISTS /opt/vc/include)
    target_include_directories(${PROJECT_NAME} PRIVATE /opt/vc/include)
    target_link_directories(${PROJECT_NAME} PRIVATE /opt/vc/lib)
    target_link_libraries(${PROJECT_NAME} mmal_core mmal_util mmal_vc_client bcm_host)
endif()