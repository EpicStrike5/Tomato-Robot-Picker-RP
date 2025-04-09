#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cstring> // For strerror
#include <cerrno>  // For errno

// Linux/POSIX Headers for Serial Port
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

// Function to configure the serial port
int setupSerialPort(const char *portName, int baudRate)
{
    int serial_port = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port < 0)
    {
        std::cerr << "Error " << errno << " opening " << portName << ": " << strerror(errno) << std::endl;
        return -1;
    }

    // Make the file descriptor blocking (remove O_NDELAY)
    fcntl(serial_port, F_SETFL, 0);

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        close(serial_port);
        return -1;
    }

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
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        std::cerr << "Warning: Unsupported baud rate " << baudRate << ". Using 115200." << std::endl;
        speed = B115200;
        break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Set Control Modes (c_cflag)
    tty.c_cflag &= ~PARENB;        // Disable parity
    tty.c_cflag &= ~CSTOPB;        // Use one stop bit
    tty.c_cflag &= ~CSIZE;         // Clear data size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // Set Local Modes (c_lflag)
    tty.c_lflag &= ~ICANON; // Disable canonical mode (line buffering)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable newline echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    // Set Input Modes (c_iflag)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

    // Set Output Modes (c_oflag)
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set VMIN and VTIME (important for non-canonical mode)
    // VMIN = 0, VTIME = 0: read() returns immediately with available data (non-blocking)
    // VMIN > 0, VTIME = 0: read() blocks until VMIN bytes are available
    // VMIN = 0, VTIME > 0: read() blocks until data arrives or VTIME deciseconds timeout
    tty.c_cc[VTIME] = 10; // Wait for up to 1 second (10 deciseconds) for data - adjust as needed
    tty.c_cc[VMIN] = 0;   // Return immediately if no data

    // Apply the settings
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        close(serial_port);
        return -1;
    }

    // Flush port to clear any existing data
    tcflush(serial_port, TCIOFLUSH);

    std::cout << "Serial port " << portName << " configured successfully." << std::endl;
    return serial_port;
}

// Function to send a command string over serial
bool sendCommand(int serial_port, const std::string &command)
{
    std::string fullCommand = command + "\n"; // Add newline terminator
    ssize_t bytes_written = write(serial_port, fullCommand.c_str(), fullCommand.length());

    if (bytes_written < 0)
    {
        std::cerr << "Error " << errno << " writing to serial port: " << strerror(errno) << std::endl;
        return false;
    }
    if (bytes_written < fullCommand.length())
    {
        std::cerr << "Warning: Only wrote " << bytes_written << " out of " << fullCommand.length() << " bytes." << std::endl;
        // Add retry logic here if needed
    }
    // Optional: Add a small delay if Arduino needs time to process
    usleep(50000); // 50ms delay - adjust or remove as needed
    return true;
}

int main()
{
    const char *port_name = "/dev/ttyACM0"; // Or ttyUSB0, etc. CHECK YOURS!
    const int baud_rate = 115200;

    int arduino_port = setupSerialPort(port_name, baud_rate);

    if (arduino_port < 0)
    {
        return 1; // Exit if serial port failed to open
    }

    std::cout << "RPi Controller Started. Enter commands (e.g., 'ElevatorTO:100,0' or 'STOP') or 'exit'." << std::endl;

    // Example Usage / Main Loop Placeholder
    // In a real application, this loop would contain your ROS logic,
    // sensor processing, decision making, etc., which then calls sendCommand.
    std::string userInput;
    while (true)
    {
        std::cout << "> ";
        std::getline(std::cin, userInput); // Read command from console for testing

        if (userInput == "exit")
        {
            break;
        }

        // Basic examples:
        if (userInput == "test_up")
        {
            sendCommand(arduino_port, "ElevatorTO:50,0"); // Move elevator to 50mm
        }
        else if (userInput == "test_down")
        {
            sendCommand(arduino_port, "ElevatorTO:0,0"); // Move elevator back to 0mm
        }
        else if (userInput == "test_extend")
        {
            sendCommand(arduino_port, "ArmTO:100,0"); // Extend arm to 100mm
        }
        else if (userInput == "test_retract")
        {
            sendCommand(arduino_port, "ArmTO:10,0"); // Retract arm to 10mm
        }
        else if (userInput == "test_forward")
        {
            sendCommand(arduino_port, "MoveTO:150,150"); // Move base forward (example speeds)
        }
        else if (userInput == "test_turn")
        {
            sendCommand(arduino_port, "MoveTO:100,-100"); // Turn base (example speeds)
        }
        else if (userInput == "stop")
        {
            sendCommand(arduino_port, "STOP"); // Send STOP command
        }
        else
        {
            // Send user input directly (allows sending custom commands like MoveTO:10,5)
            if (!userInput.empty())
            {
                sendCommand(arduino_port, userInput);
            }
        }

        // Optional: Read acknowledgment or status from Arduino (add read logic if needed)
        // char read_buf[256];
        // int n = read(arduino_port, &read_buf, sizeof(read_buf)-1);
        // if (n > 0) {
        //     read_buf[n] = '\0';
        //     std::cout << "Arduino response: " << read_buf << std::endl;
        // }

        // --- Your ROS code or other control logic would go here ---
        // This loop is just for manual command testing.
        // In a real system, you'd replace the std::cin part with
        // logic that determines the needed action and constructs the
        // appropriate command string.
        // For example:
        // if (need_to_move_up) {
        //    sendCommand(arduino_port, "ElevatorTO:150,0");
        //    need_to_move_up = false;
        // }
    }

    // Clean up
    std::cout << "Exiting. Closing serial port." << std::endl;
    sendCommand(arduino_port, "STOP"); // Send a final stop command
    usleep(100000);                    // Give it time to process
    close(arduino_port);

    return 0;
}