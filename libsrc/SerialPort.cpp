#include "SerialPort.h"
#include <fcntl.h>      // For fcntl, O_RDWR, O_NOCTTY, O_NONBLOCK
#include <unistd.h>     // For read, write, close, isatty
#include <errno.h>      // Error number definitions
#include <string.h>     // For strerror
#include <system_error> // For std::system_error
#include <utility>      // For std::move
#include <iostream>     // For cerr (error reporting)
#include <mutex>        // For std::lock_guard
#include <poll.h>       // For poll (if needed)
#include <chrono>       // For std::chrono::milliseconds
#include <thread>       // For std::this_thread::sleep_for

// Constructor: Opens port, sets O_NONBLOCK using fcntl
SerialPort::SerialPort(const std::string &port_name, speed_t baud_rate)
    : port_name_(port_name)
{

    // Open in Read/Write mode, without becoming controlling terminal
    // We'll add O_NONBLOCK later with fcntl for clarity.
    fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        throw std::system_error(errno, std::system_category(),
                                "Error opening serial port " + port_name_);
    }

    // Check if it's actually a serial device (optional but good practice)
    if (!isatty(fd))
    {
        ::close(fd);
        throw std::runtime_error("Device " + port_name_ + " is not a TTY (serial port).");
    }

    // Set to non-blocking mode AFTER successful open
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
    {
        int errsv = errno; // Save errno before close potentially changes it
        ::close(fd);
        throw std::system_error(errsv, std::system_category(), "Error getting serial port flags (fcntl F_GETFL)");
    }

    flags |= O_NONBLOCK; // Add the non-blocking flag

    if (fcntl(fd, F_SETFL, flags) == -1)
    {
        int errsv = errno;
        ::close(fd);
        throw std::system_error(errsv, std::system_category(), "Error setting serial port to non-blocking (fcntl F_SETFL)");
    }

    // Now configure termios settings
    try
    {
        configurePort(baud_rate); // Configure termios settings
        is_open_ = true;
        tcflush(fd, TCIOFLUSH); // Flush any stale data in buffers
    }
    catch (...)
    {
        // Ensure port is closed if configurePort throws
        if (fd >= 0)
        {
            ::close(fd);
        }
        throw; // Re-throw the exception
    }

    // Use std::cerr for initial status messages as stdout might be used differently
    std::cerr << "Serial port " << port_name_ << " opened successfully (non-blocking)." << std::endl;
}

// Destructor
SerialPort::~SerialPort()
{
    closePort();
}

// Move Constructor
SerialPort::SerialPort(SerialPort &&other) noexcept
    : fd(other.fd), port_name_(std::move(other.port_name_)),
      is_open_(other.is_open_), tty_config_(other.tty_config_)
// Mutex is not moved; the new object gets its own default state.
{
    // Prevent the moved-from object's destructor from closing the file descriptor
    other.fd = -1;
    other.is_open_ = false;
}

// Move Assignment Operator
SerialPort &SerialPort::operator=(SerialPort &&other) noexcept
{
    if (this != &other)
    {
        // Lock guard ensures mutex safety even during the move
        std::lock_guard<std::mutex> this_lock(port_mutex_); // Lock current mutex before modifying state

        closePort(); // Close the current port if open

        // Move resources (no mutex needed for 'other' if we invalidate it)
        fd = other.fd;
        port_name_ = std::move(other.port_name_);
        is_open_ = other.is_open_;
        tty_config_ = other.tty_config_;

        // Prevent the moved-from object's destructor from closing the file descriptor
        other.fd = -1;
        other.is_open_ = false;
    }
    return *this;
}

// configurePort: Sets RAW mode and VMIN=0, VTIME=0 for non-blocking
void SerialPort::configurePort(speed_t baud_rate)
{
    // Get current attributes
    if (tcgetattr(fd, &tty_config_) != 0)
    {
        throw std::system_error(errno, std::system_category(), "Error from tcgetattr");
    }

    // --- Set flags for RAW mode ---
    // Control Modes (c_cflag)
    tty_config_.c_cflag &= ~PARENB;        // No parity
    tty_config_.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty_config_.c_cflag &= ~CSIZE;         // Clear data size bits
    tty_config_.c_cflag |= CS8;            // 8 data bits
    tty_config_.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty_config_.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem status lines

    // Local Modes (c_lflag)
    tty_config_.c_lflag &= ~ICANON; // Disable canonical mode (line-by-line)
    tty_config_.c_lflag &= ~ECHO;   // Disable echoing input characters
    tty_config_.c_lflag &= ~ECHOE;  // Disable echo erase
    tty_config_.c_lflag &= ~ECHONL; // Disable echo newline
    tty_config_.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT, SUSP signals

    // Input Modes (c_iflag)
    tty_config_.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Disable software flow control
    tty_config_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special input handling

    // Output Modes (c_oflag)
    tty_config_.c_oflag &= ~OPOST; // Disable implementation-defined output processing
    tty_config_.c_oflag &= ~ONLCR; // Disable map NL to CR-NL

    // --- Set VMIN and VTIME for Non-blocking read() ---
    // With O_NONBLOCK set on the file descriptor:
    // VMIN = 0, VTIME = 0: read() returns immediately.
    // Returns bytes read (1 to n) if data is available.
    // Returns 0 if EOF (rare for serial).
    // Returns -1 with errno EAGAIN or EWOULDBLOCK if no data is available.
    tty_config_.c_cc[VMIN] = 0;
    tty_config_.c_cc[VTIME] = 0;

    // Set Baud Rate
    if (cfsetispeed(&tty_config_, baud_rate) != 0)
    {
        throw std::system_error(errno, std::system_category(), "Error setting input baud rate");
    }
    if (cfsetospeed(&tty_config_, baud_rate) != 0)
    {
        throw std::system_error(errno, std::system_category(), "Error setting output baud rate");
    }

    // Apply the attributes
    if (tcsetattr(fd, TCSANOW, &tty_config_) != 0)
    {
        throw std::system_error(errno, std::system_category(), "Error from tcsetattr");
    }
}

// writeString: Calls writeBytes
void SerialPort::writeString(const std::string &data)
{
    writeBytes(reinterpret_cast<const uint8_t *>(data.c_str()), data.length());
}

// writeBytes: Includes mutex lock
void SerialPort::writeBytes(const uint8_t *buffer, size_t size)
{
    std::lock_guard<std::mutex> lock(port_mutex_); // Lock the mutex
    if (!is_open_ || fd < 0)
    {
        throw std::runtime_error("Serial port is not open or invalid for writing.");
    }

    ssize_t total_bytes_written = 0;
    while (total_bytes_written < static_cast<ssize_t>(size))
    {
        ssize_t bytes_written = ::write(fd, buffer + total_bytes_written, size - total_bytes_written);

        if (bytes_written < 0)
        {
            // EAGAIN/EWOULDBLOCK might occur even for write in non-blocking, means try again later
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Avoid busy-wait on write
                continue;                                                  // Retry write
            }
            else
            {
                throw std::system_error(errno, std::system_category(), "Error writing to serial port");
            }
        }
        if (bytes_written == 0)
        {
            // Should not happen for regular files or serial ports, but handle defensively
            std::cerr << "Warning: write() returned 0, potential issue." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        total_bytes_written += bytes_written;
    }

    // Optional: Ensure data is physically transmitted before returning
    // if (tcdrain(fd) == -1) {
    //     std::cerr << "Warning: tcdrain failed: " << strerror(errno) << std::endl;
    // }

} // Mutex automatically unlocked here by lock_guard destructor

// readBytes: Handles non-blocking reads and EAGAIN/EWOULDBLOCK
ssize_t SerialPort::readBytes(uint8_t *buffer, size_t buffer_size)
{
    std::lock_guard<std::mutex> lock(port_mutex_); // Lock mutex
    if (!is_open_ || fd < 0)
    {
        throw std::runtime_error("Serial port is not open or invalid for reading.");
    }
    if (buffer == nullptr || buffer_size == 0)
    {
        throw std::invalid_argument("Read buffer is null or buffer_size is zero.");
    }

    // Perform the non-blocking read
    ssize_t bytes_read = ::read(fd, buffer, buffer_size);

    if (bytes_read < 0)
    {
        // In non-blocking mode, EAGAIN or EWOULDBLOCK simply means "no data available now"
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0; // Indicate no data was read this time (not an error)
        }
        else
        {
            // It's a real error
            throw std::system_error(errno, std::system_category(), "Error reading from serial port");
        }
    }
    // If bytes_read == 0: Could mean EOF (e.g., device disconnected). Pass it up.
    // If bytes_read > 0: Data was read successfully.

    return bytes_read;
} // Mutex unlocked

// isOpen: Checks the flag
bool SerialPort::isOpen() const
{
    // Reading bool should be atomic enough, mutex optional for strictness
    // std::lock_guard<std::mutex> lock(port_mutex_);
    return is_open_;
}

// closePort: Includes mutex lock
void SerialPort::closePort()
{
    std::lock_guard<std::mutex> lock(port_mutex_); // Lock the mutex
    if (is_open_ && fd >= 0)
    {
        std::cerr << "Closing serial port " << port_name_ << std::endl;
        if (::close(fd) < 0)
        {
            // Log error, but destructor shouldn't throw
            std::cerr << "Error closing serial port " << port_name_ << ": " << strerror(errno) << std::endl;
        }
    }
    fd = -1;
    is_open_ = false;
} // Mutex unlocked

// intToSpeedT: Helper function
speed_t SerialPort::intToSpeedT(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    // Add other common rates if needed
    default:
        throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud));
    }
}