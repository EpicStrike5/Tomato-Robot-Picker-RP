#include "SerialPort.hpp"

#include <iostream>
#include <cstring>   // For strerror, memset
#include <cerrno>    // For errno
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <unistd.h>  // write(), read(), close(), usleep
#include <poll.h>    // For poll()
#include <stdexcept> // Optional: for throwing exceptions

// --- Constructor ---
SerialPort::SerialPort() : serial_fd_(-1), is_open_(false), baud_rate_(0)
{
    // Initialize buffer or other members if needed
}

// --- Destructor ---
SerialPort::~SerialPort()
{
    if (is_open_)
    {
        closePort();
    }
}

// --- openPort ---
bool SerialPort::openPort(const std::string &portName, int baudRate)
{
    if (is_open_)
    {
        std::cerr << "Warning: Port " << port_name_ << " already open. Closing first." << std::endl;
        closePort();
    }

    port_name_ = portName;
    baud_rate_ = baudRate;

    // O_RDWR: Read/Write access
    // O_NOCTTY: Don't become the process's controlling terminal
    // O_NDELAY: Non-blocking open (we'll make it blocking later with fcntl)
    // O_EXCL: Ensure that we are creating the file (fails if it already exists - not typical for devices)
    // O_NONBLOCK: POSIX standard non-blocking I/O
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0)
    {
        std::cerr << "Error " << errno << " opening " << port_name_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    // Make the file descriptor blocking for reads by default (can use timeout in readSerial)
    // We remove O_NONBLOCK set during open
    int flags = fcntl(serial_fd_, F_GETFL, 0);
    if (flags == -1)
    {
        std::cerr << "Error " << errno << " getting flags for " << port_name_ << ": " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    if (fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK) == -1)
    {
        std::cerr << "Error " << errno << " setting blocking mode for " << port_name_ << ": " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Get current settings and store old ones (optional)
    if (tcgetattr(serial_fd_, &tty_settings_) != 0)
    {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    tty_old_settings_ = tty_settings_; // Save old settings

    if (!configureTermios(baudRate))
    {
        // Restore old settings if configuration failed
        tcsetattr(serial_fd_, TCSANOW, &tty_old_settings_);
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Flush port to clear any existing data
    if (!flushIO())
    {
        // Warning or error? Depends on how critical flushing is.
        std::cerr << "Warning: Failed to flush IO buffers on port open." << std::endl;
    }

    is_open_ = true;
    std::cout << "Serial port " << port_name_ << " opened successfully." << std::endl;
    return true;
}

// --- configureTermios (Private Helper) ---
bool SerialPort::configureTermios(int baudRate)
{
    // Set Baud Rate
    speed_t speed;
    switch (baudRate)
    {
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break; // Added common rate
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break; // Added common rate
    default:
        std::cerr << "Warning: Unsupported baud rate " << baudRate << ". Using 115200." << std::endl;
        speed = B115200;
        baud_rate_ = 115200; // Update the stored rate
        break;
    }
    if (cfsetospeed(&tty_settings_, speed) != 0 || cfsetispeed(&tty_settings_, speed) != 0)
    {
        std::cerr << "Error " << errno << " setting baud rate: " << strerror(errno) << std::endl;
        return false;
    }

    // Set Control Modes (c_cflag)
    tty_settings_.c_cflag &= ~PARENB;        // Disable parity
    tty_settings_.c_cflag &= ~CSTOPB;        // Use one stop bit
    tty_settings_.c_cflag &= ~CSIZE;         // Clear data size bits
    tty_settings_.c_cflag |= CS8;            // 8 data bits
    tty_settings_.c_cflag &= ~CRTSCTS;       // Disable hardware flow control
    tty_settings_.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // Set Local Modes (c_lflag) - Non-Canonical mode
    tty_settings_.c_lflag &= ~ICANON; // Disable canonical mode (line buffering)
    tty_settings_.c_lflag &= ~ECHO;   // Disable echo
    tty_settings_.c_lflag &= ~ECHOE;  // Disable erasure
    tty_settings_.c_lflag &= ~ECHONL; // Disable newline echo
    tty_settings_.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    // Set Input Modes (c_iflag)
    tty_settings_.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty_settings_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

    // Set Output Modes (c_oflag)
    tty_settings_.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty_settings_.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set VMIN and VTIME (for non-canonical mode)
    // VMIN = 0, VTIME = 0: Non-blocking read. read() returns immediately.
    // VMIN > 0, VTIME = 0: Blocking read until VMIN bytes received.
    // VMIN = 0, VTIME > 0: Read with timeout. Blocks until VTIME * 0.1 seconds passes or data is received.
    // VMIN > 0, VTIME > 0: Read with inter-byte timeout. Timer starts after first byte.
    // We will handle timeouts using poll() instead of VTIME, so set VMIN/VTIME for non-blocking reads within termios
    tty_settings_.c_cc[VTIME] = 0; // No timeout within termios read itself
    tty_settings_.c_cc[VMIN] = 0;  // Read returns immediately with available bytes

    // Apply the settings
    if (tcsetattr(serial_fd_, TCSANOW, &tty_settings_) != 0)
    {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

// --- closePort ---
void SerialPort::closePort()
{
    if (is_open_)
    {
        // Optional: Restore old terminal settings
        // tcsetattr(serial_fd_, TCSANOW, &tty_old_settings_);

        if (close(serial_fd_) != 0)
        {
            std::cerr << "Error " << errno << " closing port " << port_name_ << ": " << strerror(errno) << std::endl;
        }
        std::cout << "Serial port " << port_name_ << " closed." << std::endl;
        is_open_ = false;
        serial_fd_ = -1;
        port_name_ = "";
        baud_rate_ = 0;
        read_buffer_.clear(); // Clear internal buffer on close
    }
}

// --- isOpen ---
bool SerialPort::isOpen() const
{
    return is_open_;
}

// --- writeSerial ---
ssize_t SerialPort::writeSerial(const void *data, size_t len)
{
    if (!is_open_)
    {
        std::cerr << "Error: Port not open for writing." << std::endl;
        return -1;
    }
    ssize_t bytes_written = write(serial_fd_, data, len);
    if (bytes_written < 0)
    {
        std::cerr << "Error " << errno << " writing to serial port " << port_name_ << ": " << strerror(errno) << std::endl;
    }
    else if (bytes_written < len)
    {
        std::cerr << "Warning: Only wrote " << bytes_written << " out of " << len << " bytes to " << port_name_ << "." << std::endl;
        // Consider adding retry logic here if needed
    }
    return bytes_written;
}

// --- writeString ---
bool SerialPort::writeString(const std::string &data)
{
    // Note: This now sends the string *exactly* as provided.
    // The caller needs to add '\n' if required by the protocol.
    return writeSerial(data.c_str(), data.length()) == (ssize_t)data.length();
}

// --- readSerial ---
ssize_t SerialPort::readSerial(void *buffer, size_t len, int timeout_ms)
{
    if (!is_open_)
    {
        std::cerr << "Error: Port not open for reading." << std::endl;
        return -1;
    }
    if (len == 0)
    {
        return 0; // Nothing to read
    }

    struct pollfd fds[1];
    fds[0].fd = serial_fd_;
    fds[0].events = POLLIN; // Check for data to read

    // poll() modifies timeout value on some systems, so pass a copy if needed
    int poll_timeout = (timeout_ms < 0) ? -1 : timeout_ms; // -1 for infinite wait

    int poll_ret = poll(fds, 1, poll_timeout);

    if (poll_ret < 0)
    {
        std::cerr << "Error " << errno << " polling serial port " << port_name_ << ": " << strerror(errno) << std::endl;
        return -1; // Poll error
    }
    else if (poll_ret == 0)
    {
        return 0; // Timeout
    }
    else
    {
        // Data is available
        if (fds[0].revents & POLLIN)
        {
            ssize_t num_bytes = read(serial_fd_, buffer, len);
            if (num_bytes < 0)
            {
                // EWOULDBLOCK or EAGAIN might mean no data ready yet (though poll said yes), treat as 0 bytes read?
                // Or a genuine read error
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    return 0; // Or retry? Let's return 0 as no data was actually read this call.
                }
                else
                {
                    std::cerr << "Error " << errno << " reading from serial port " << port_name_ << ": " << strerror(errno) << std::endl;
                    return -1; // Read error
                }
            }
            // num_bytes >= 0 is success (could be 0 if VMIN/VTIME caused read to return 0)
            return num_bytes;
        }
        else if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            // An error occurred on the file descriptor
            std::cerr << "Error event " << fds[0].revents << " on serial port " << port_name_ << std::endl;
            // Consider closing the port or signalling a major error
            return -1; // Indicate error
        }
    }
    return -1; // Should not be reached, but covers unexpected cases
}

// --- readLine ---
std::string SerialPort::readLine(int timeout_ms)
{
    if (!is_open_)
    {
        std::cerr << "Error: Port not open for reading line." << std::endl;
        return "";
    }

    // Check internal buffer first
    size_t newline_pos = read_buffer_.find('\n');
    if (newline_pos != std::string::npos)
    {
        std::string line = read_buffer_.substr(0, newline_pos);
        read_buffer_.erase(0, newline_pos + 1); // Remove line and newline char
        return line;
    }

    // If no complete line in buffer, try reading more data
    char temp_buf[256]; // Read in chunks
    int elapsed_time = 0;
    const int poll_interval = 10; // Check for data every 10ms

    while (timeout_ms < 0 || elapsed_time < timeout_ms)
    {
        int current_timeout = (timeout_ms < 0) ? -1 : std::max(0, timeout_ms - elapsed_time);
        // If timeout_ms is positive, calculate remaining time for poll
        // If timeout_ms is negative (infinite), use -1 for poll
        // Limit poll interval slightly to avoid busy-wait if timeout_ms is small
        if (timeout_ms > 0)
        {
            current_timeout = std::min(poll_interval, current_timeout);
        }

        ssize_t bytes_read = readSerial(temp_buf, sizeof(temp_buf) - 1, current_timeout);

        if (bytes_read < 0)
        { // Error
            // Error message already printed by readSerial
            return ""; // Return empty string on error
        }

        if (bytes_read > 0)
        {
            temp_buf[bytes_read] = '\0';               // Null-terminate C-string style
            read_buffer_.append(temp_buf, bytes_read); // Append to internal buffer

            // Check again for newline
            newline_pos = read_buffer_.find('\n');
            if (newline_pos != std::string::npos)
            {
                std::string line = read_buffer_.substr(0, newline_pos);
                // Handle potential '\r\n' by checking if '\r' precedes '\n'
                if (newline_pos > 0 && line.back() == '\r')
                {
                    line.pop_back(); // Remove trailing '\r'
                }
                read_buffer_.erase(0, newline_pos + 1); // Remove line and newline char
                return line;
            }
        }
        // else bytes_read == 0 (timeout on this read attempt)

        if (timeout_ms > 0)
        {
            elapsed_time += (bytes_read >= 0) ? current_timeout : poll_interval; // Increment elapsed time
            // If bytes_read < 0 (error), we break anyway. If bytes_read == 0 (timeout), we increment.
            // If bytes_read > 0, we still consumed time polling/reading.
            if (elapsed_time >= timeout_ms)
                break; // Exit loop if total timeout exceeded
        }
        // If timeout_ms < 0 (infinite), loop continues until line found or error
    }

    // Timeout occurred before newline was found, or infinite wait was interrupted by error
    // Return whatever might be in the buffer (partial line) or empty string?
    // Standard behavior is often to return empty on timeout.
    return ""; // Timeout or error
}

// --- flushIO ---
bool SerialPort::flushIO()
{
    if (!is_open_)
    {
        std::cerr << "Error: Port not open for flushing." << std::endl;
        return false;
    }
    // TCIOFLUSH: Flushes data received but not read, and data written but not transmitted.
    // TCIFLUSH: Flushes data received but not read.
    // TCOFLUSH: Flushes data written but not transmitted.
    if (tcflush(serial_fd_, TCIOFLUSH) == -1)
    {
        std::cerr << "Error " << errno << " flushing serial port " << port_name_ << ": " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}