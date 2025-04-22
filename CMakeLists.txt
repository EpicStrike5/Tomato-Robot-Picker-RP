# CMake minimum version required
cmake_minimum_required(VERSION 3.10)

# Project Name and Language
project(TomatoRobotPicker LANGUAGES CXX)

# Set C++ standard (e.g., C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Library Definition ---
add_library(serialport libsrc/SerialPort.cpp)

# Specify that the public include directory for the 'serialport' library
target_include_directories(serialport PUBLIC ${CMAKE_SOURCE_DIR}/include)


# --- Executable Definition ---
add_executable(main src/main.cpp)

# Link the main executable against the serialport library
target_link_libraries(main PRIVATE serialport)

# Optional: Place the executable directly in the build folder root
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Optional: Print status message
message(STATUS "Configuring TomatoRobotPicker project...")
message(STATUS "  Executable target: main")
message(STATUS "  Library target: serialport")