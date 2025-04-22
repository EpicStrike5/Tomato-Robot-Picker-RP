#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <string>
#include <vector>
#include <termios.h> // For termios structure, needed in header for private member

class SerialPort
{
public:
    // Constructor: Initializes but doesn't open the port
    SerialPort();

    // Destructor: Ensures the port is closed
    virtual ~SerialPort();

    // Opens and configures the serial port
    // Returns true on success, false on failure
    bool openPort(const std::string &portName, int baudRate);

    // Closes the serial port
    void closePort();

    // Checks if the port is open
    bool isOpen() const;

    // Writes raw data to the serial port
    // Returns number of bytes written, or -1 on error
    ssize_t writeSerial(const void *data, size_t len);

    // Writes a string to the serial port (doesn't automatically add newline)
    // Returns true on success, false on failure
    bool writeString(const std::string &data);

    // Reads raw data from the serial port
    // buffer: Pointer to the buffer to store data
    // len: Maximum number of bytes to read
    // timeout_ms: Timeout in milliseconds.
    //             0 = non-blocking read (return immediately)
    //             -1 = blocking read (wait indefinitely)
    //             >0 = wait up to timeout_ms milliseconds
    // Returns number of bytes read, 0 on timeout, or -1 on error
    ssize_t readSerial(void *buffer, size_t len, int timeout_ms = 0);

    // Reads a line (up to newline '\n') or until timeout
    // timeout_ms: Timeout in milliseconds to wait for a complete line.
    // Returns the line read (without newline), or an empty string on timeout/error.
    std::string readLine(int timeout_ms = 100); // Default timeout 100ms

    // Flushes the input/output buffers
    bool flushIO();

    // **** ADD THIS GETTER ****
    // Returns the raw file descriptor for use with poll/select etc.
    // Returns -1 if the port is not open.
    int getFD() const;

private:
    int serial_fd_ = -1; // File descriptor for the serial port
    bool is_open_ = false;
    std::string port_name_;
    int baud_rate_;
    struct termios tty_settings_;     // Current port settings
    struct termios tty_old_settings_; // To restore settings on close (optional but good practice)
    std::string read_buffer_;         // Internal buffer for readLine

    // Helper function to configure termios settings
    bool configureTermios(int baudRate);

    // Prevent copying
    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;
};

#endif // SERIALPORT_HPP