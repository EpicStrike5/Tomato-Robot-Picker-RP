# ======================================================
# CMake Configuration for TomatoRobotPicker
# ======================================================

# --- Project Setup ---
cmake_minimum_required(VERSION 3.10)
project(TomatoRobotPicker LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# SET(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++) # REMOVED - Usually not needed when building on target
message(STATUS "Configuring ${PROJECT_NAME}...")
message(STATUS "CXX Standard: ${CMAKE_CXX_STANDARD}")

# --- Find External Packages ---

# 1. Find OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}")
    set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}) # Store for later use
    message(STATUS "OpenCV Include Dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV required, but not found.")
endif()

# 2. Find ONNX Runtime
find_package(onnxruntime REQUIRED)
if(NOT onnxruntime_FOUND)
    message(FATAL_ERROR "ONNX Runtime required, but not found.")
else()
    message(STATUS "Found ONNX Runtime")
    # You can check these variables if needed for debugging, but the target should handle paths
    message(STATUS "ONNX Runtime Include Dirs (from find script): ${onnxruntime_INCLUDE_DIRS}")
    message(STATUS "ONNX Runtime Library Path (from find script): ${onnxruntime_LIBRARY}")
endif()

# REMOVED - Redundant/Incorrect: include_directories(${onnxruntime_INCLUDE_DIRS}/onnxruntime/include)

# --- Define Internal Libraries ---

# 1. Serial Port Library
add_library(robot_serial_lib STATIC
    libsrc/SerialPort.cpp
)
target_include_directories(robot_serial_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include # For SerialPort.h
)
message(STATUS "Defined internal library: robot_serial_lib")


# --- Define Executable ---

add_executable(main
    src/main.cpp
)
message(STATUS "Defined executable: main")


# --- Configure Include Directories for Executable ---

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # Project headers (YOLO12.hpp, SerialPort.h)
    ${OpenCV_INCLUDE_DIRS}               # OpenCV headers
    # ONNX Runtime includes are handled by linking the 'onnxruntime' target below
)
message(STATUS "Configured include directories for target 'main'")


# --- Configure Linking for Executable ---

target_link_libraries(main PRIVATE
    # Internal libraries
    robot_serial_lib

    # External libraries
    ${OpenCV_LIBS}      # Link OpenCV using the variable
    onnxruntime         # Link ONNX Runtime using the IMPORTED TARGET <<< CORRECTED BACK >>>
)
message(STATUS "Configured linking for target 'main'")


# --- Final Output ---
message(STATUS "--- ${PROJECT_NAME} Configuration Complete ---")