cmake_minimum_required(VERSION 3.16)
project(puck_tracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_HEADLESS "Build without GUI support" ON)
option(USE_ABB "Enable ABB integration if abb_comm is present" ON)

if(USE_ABB)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ABB=1)
endif()


# --- OpenCV
if(BUILD_HEADLESS)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)
else()
    find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui video videoio)
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- Źródła aplikacji
set(SOURCES
    src/capture.cpp
    src/detector.cpp
    src/tracker.cpp
    src/utils.cpp
)



if(BUILD_HEADLESS)
    # zamiast main_headless.cpp dodajemy http_puck_tracker.cpp
    list(APPEND SOURCES src/http_puck_tracker.cpp)
else()
    list(APPEND SOURCES src/main.cpp src/overlay.cpp)
endif()

# --- Główna aplikacja
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
        opencv_video        # <-- dodaj jawnie, gdyby ${OpenCV_LIBS} nie dodało
        Threads::Threads
        yaml-cpp
        atomic
)

# --- RPi camera (legacy stack, jeśli występuje)
if(EXISTS /opt/vc/include)
    target_include_directories(${PROJECT_NAME} PRIVATE /opt/vc/include)
    target_link_directories(${PROJECT_NAME} PRIVATE /opt/vc/lib)
    target_link_libraries(${PROJECT_NAME} PRIVATE mmal_core mmal_util mmal_vc_client bcm_host)
endif()

# =========================
#  ABB (opcjonalnie)
# =========================
set(ABB_COMM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/open_abb/abb_comm)

if(USE_ABB AND EXISTS ${ABB_COMM_DIR})
    message(STATUS "ABB: found abb_comm at ${ABB_COMM_DIR}")
    file(GLOB OPEN_ABB_COMM_SOURCES
        ${ABB_COMM_DIR}/*.cpp
    )
    add_library(open_abb_comm ${OPEN_ABB_COMM_SOURCES})
    target_include_directories(open_abb_comm PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/external/open_abb
        ${ABB_COMM_DIR}
    )

    add_library(abb_robot
        src/robot/abb/AbbController.cpp
        src/robot/abb/RobotStrikePlanner.cpp
    )
    target_include_directories(abb_robot PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/external/open_abb
        ${ABB_COMM_DIR}
    )
    target_link_libraries(abb_robot PUBLIC open_abb_comm)

    target_link_libraries(${PROJECT_NAME} PRIVATE abb_robot)
else()
    message(STATUS "ABB: abb_comm NOT found or USE_ABB=OFF — building without ABB support")
endif()
