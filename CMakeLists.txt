cmake_minimum_required(VERSION 3.16)
project(puck_tracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization flags for ARM
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -ffast-math -funroll-loops -ftree-vectorize")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mcpu=cortex-a72")
    endif()
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -ffast-math")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")

# Find packages
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui videoio video)
find_package(Threads REQUIRED)
find_package(PkgConfig)

# Optional packages
pkg_check_modules(GSTREAMER gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP gstreamer-app-1.0)
find_package(Eigen3 3.3)

# Check for libcamera
pkg_check_modules(LIBCAMERA libcamera)

# Source files
set(SOURCES
    src/main.cpp
    src/capture.cpp
    src/detector.cpp
    src/tracker.cpp
    src/overlay.cpp
    src/utils.cpp
)

# Headers
set(HEADERS
    src/capture.hpp
    src/detector.hpp
    src/tracker.hpp
    src/overlay.hpp
    src/utils.hpp
    src/lock_free_queue.hpp
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Threads::Threads
    atomic
)

# Optional dependencies
if(GSTREAMER_FOUND AND GSTREAMER_APP_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_GSTREAMER)
    target_include_directories(${PROJECT_NAME} PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER_APP_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES})
endif()

if(LIBCAMERA_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_LIBCAMERA)
    target_include_directories(${PROJECT_NAME} PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${LIBCAMERA_LIBRARIES})
endif()

if(Eigen3_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_EIGEN)
    target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config/config.yaml DESTINATION etc/puck_tracker)
install(FILES scripts/launch.sh DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "GStreamer support: ${GSTREAMER_FOUND}")
message(STATUS "libcamera support: ${LIBCAMERA_FOUND}")
message(STATUS "Eigen3 support: ${Eigen3_FOUND}")