#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <termios.h> // Contains POSIX terminal control definitions
#include <stdexcept> // For standard exceptions
#include <mutex>     // Include mutex header
#include <unistd.h>  // For ssize_t

class SerialPort
{
public:
    // Constructor: Opens and configures the serial port (now non-blocking).
    // Throws std::runtime_error or std::system_error on failure.
    SerialPort(const std::string &port_name, speed_t baud_rate = B9600);

    // Destructor: Closes the serial port if open.
    ~SerialPort();

    // Delete copy constructor and assignment operator to prevent copying
    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    // Move constructor and assignment operator
    SerialPort(SerialPort &&other) noexcept;
    SerialPort &operator=(SerialPort &&other) noexcept;

    // Write data to the serial port.
    // Throws std::system_error on failure.
    void writeString(const std::string &data);
    void writeBytes(const uint8_t *buffer, size_t size);

    // Read data from the serial port (non-blocking).
    // Returns >0 bytes read if data available.
    // Returns 0 if no data available immediately (EAGAIN/EWOULDBLOCK).
    // Throws std::system_error on other read errors.
    ssize_t readBytes(uint8_t *buffer, size_t buffer_size);

    // Check if the port is open.
    bool isOpen() const;

private:
    int fd = -1; // File descriptor for the serial port
    std::string port_name_;
    bool is_open_ = false;
    struct termios tty_config_;     // To store terminal settings
    mutable std::mutex port_mutex_; // Mutex to protect access to fd

    // Configures the serial port settings (baud rate, 8N1, raw mode, VMIN/VTIME for non-blocking).
    void configurePort(speed_t baud_rate);

    // Closes the serial port.
    void closePort();

    // Helper to convert integer baud to speed_t
    speed_t intToSpeedT(int baud);
};

#endif // SERIALPORT_H