# CMake minimum version required
cmake_minimum_required(VERSION 3.10)

# Project Name and Language
project(TomatoRobotPicker LANGUAGES CXX)

# Set C++ standard (e.g., C++17)
set(CMAKE_CXX_STANDARD 17) # Or 11, 14 as needed
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Find External Packages using pkg-config ---
find_package(PkgConfig REQUIRED) 
pkg_check_modules(SerialPort REQUIRED libserialport) 
message(STATUS "Found libserialport via pkg-config:") 
message(STATUS "  Include Dirs: ${SerialPort_INCLUDE_DIRS}")
message(STATUS "  Libraries: ${SerialPort_LIBRARIES}")


# --- Your Library Definition ---
set(ROBOT_SERIAL_LIB_NAME robot_serial_lib) 
# *** CORRECTED: Only list SOURCE files (.cpp) for add_library ***
add_library(${ROBOT_SERIAL_LIB_NAME} STATIC 
    libsrc/SerialPort.cpp 
)

# Specify include directories needed by your library and consumers
# PUBLIC means both the library itself and consumers (like main) can see these paths.
# *** CORRECTED: Point to your 'include' directory and the libserialport includes ***
target_include_directories(${ROBOT_SERIAL_LIB_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # <<< Your header directory
    ${SerialPort_INCLUDE_DIRS}          # <<< Include paths found by pkg-config
)


# --- Executable Definition ---
# Assuming main.cpp is in src/
add_executable(main src/main.cpp)

# --- Linking ---
# Link the main executable against YOUR library AND the system's libserialport library
# Use the variables provided by pkg_check_modules
target_link_libraries(main PRIVATE 
    ${ROBOT_SERIAL_LIB_NAME}  # Link against your wrapper library
    ${SerialPort_LIBRARIES}   # <<< Link against library name(s) found by pkg-config
)


# --- Status Messages ---
message(STATUS "Configuring TomatoRobotPicker project...")
message(STATUS "  Executable target: main")
message(STATUS "  Library target: ${ROBOT_SERIAL_LIB_NAME}")
message(STATUS "  Linking main against: ${ROBOT_SERIAL_LIB_NAME} and libraries [${SerialPort_LIBRARIES}]")