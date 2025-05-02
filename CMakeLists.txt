# ======================================================
# CMake Configuration for TomatoRobotPicker
# ======================================================

# --- Project Setup ---
cmake_minimum_required(VERSION 3.10)
project(TomatoRobotPicker LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
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
endif()

# 3. Find Librealsense2
find_package(realsense2 REQUIRED)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "Librealsense2 required, but not found.")
else()
     message(STATUS "Found Librealsense2")
     # Add include directories provided by find_package
     list(APPEND EXTRA_INCLUDE_DIRS ${realsense2_INCLUDE_DIRS})
endif()

# 4. Find Eigen3
find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 required, but not found.")
else()
    message(STATUS "Found Eigen3")
    # Add include directories provided by find_package
    list(APPEND EXTRA_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


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
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # Project headers (YOLO12.hpp, SerialPort.h, cv-helpers.hpp)
    ${OpenCV_INCLUDE_DIRS}               # OpenCV headers
    ${EXTRA_INCLUDE_DIRS}                # Include dirs from realsense2, eigen3 etc.
    # ONNX Runtime includes are handled by linking the 'onnxruntime' target below
)
message(STATUS "Configured include directories for target 'main'")


# --- Configure Linking for Executable ---

target_link_libraries(main PRIVATE
    # Internal libraries
    robot_serial_lib

    # External libraries
    ${OpenCV_LIBS}      # Link OpenCV using the variable
    onnxruntime         # Link ONNX Runtime using the IMPORTED TARGET
    realsense2          # Link Librealsense2 using the target from find_package
    # Eigen3 is typically header-only, linking might not be needed,
    # but find_package ensures headers are found.
    # Eigen3::Eigen # Optional: Use if find_package defines this target
)
message(STATUS "Configured linking for target 'main'")


# --- Final Output ---
message(STATUS "--- ${PROJECT_NAME} Configuration Complete ---")