#include "../include/SerialPort.hpp" // Include the library header
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>  // For usleep, STDIN_FILENO
#include <poll.h>    // For poll()
#include <stdexcept> // For runtime_error (optional error handling)

int main()
{
    const char *port_name = "/dev/ttyACM0"; // CHECK YOURS!
    const int baud_rate = 115200;

    SerialPort arduinoPort; // Create a SerialPort object

    // Open and configure the port
    if (!arduinoPort.openPort(port_name, baud_rate))
    {
        std::cerr << "Failed to open serial port." << std::endl;
        return 1;
    }

    // Get the file descriptor for the serial port
    int serial_fd = arduinoPort.getFD();
    if (serial_fd < 0)
    {
        std::cerr << "Failed to get serial port file descriptor." << std::endl;
        return 1; // Should not happen if openPort succeeded, but check anyway
    }

    std::cout << "RPi Controller Started. Using SerialPort library and poll()." << std::endl;
    std::cout << "Enter commands (e.g., 'ElevatorTO:100,0' or 'STOP') or 'exit'." << std::endl;
    std::cout.flush(); // Ensure prompt appears before potential delay

    // Give Arduino some time to boot/initialize
    usleep(2000000);       // 2 seconds delay - Adjust as necessary
    arduinoPort.flushIO(); // Flush any garbage data

    // Prepare structures for poll()
    struct pollfd fds[2];

    // Monitor standard input (for user commands)
    fds[0].fd = STDIN_FILENO; // File descriptor for stdin (usually 0)
    fds[0].events = POLLIN;   // Check for data to read

    // Monitor serial port (for Arduino messages)
    fds[1].fd = serial_fd;  // File descriptor from SerialPort object
    fds[1].events = POLLIN; // Check for data to read

    bool running = true;
    std::string userInput;

    while (running)
    {
        // Display prompt only if needed (less clutter)
        // std::cout << "> ";
        // std::cout.flush();

        // Wait for events on either stdin or serial port
        // Timeout set to 100ms. Adjust as needed.
        // -1 would wait indefinitely until an event occurs.
        // 0 would return immediately (busy-wait, not recommended here).
        int poll_ret = poll(fds, 2, 100); // Wait up to 100 milliseconds

        if (poll_ret < 0)
        {
            perror("poll error"); // Print system error message
            break;                // Exit on poll error
        }
        else if (poll_ret == 0)
        {
            // Timeout occurred - no data available on either fd
            // You can add periodic tasks here if needed
            continue;
        }
        else
        {
            // --- Check Standard Input (User Commands) ---
            if (fds[0].revents & POLLIN)
            {
                if (std::getline(std::cin, userInput))
                {
                    if (userInput == "exit")
                    {
                        running = false; // Signal loop termination
                        continue;        // Skip sending the "exit" command itself
                    }

                    // Process and send the command
                    if (!userInput.empty())
                    {
                        std::string commandToSend = userInput + "\n";
                        if (!arduinoPort.writeString(commandToSend))
                        {
                            std::cerr << "Failed to send command: " << userInput << std::endl;
                            if (!arduinoPort.isOpen())
                            {
                                std::cerr << "Serial port closed. Exiting." << std::endl;
                                running = false;
                            }
                        }
                        else
                        {
                            std::cout << "Sent: " << userInput << std::endl; // User feedback
                        }
                    }
                }
                else // Error or EOF on stdin
                {
                    if (std::cin.eof())
                    {
                        std::cout << "EOF detected on stdin, exiting." << std::endl;
                    }
                    else
                    {
                        std::cerr << "Error reading stdin." << std::endl;
                    }
                    running = false; // Exit loop on stdin error/EOF
                }
            }
            else if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                std::cerr << "Error condition on stdin. Exiting." << std::endl;
                running = false;
            }

            // --- Check Serial Port (Arduino Messages) ---
            if (fds[1].revents & POLLIN)
            {
                // Data is available, read it using the existing method
                // Using a timeout of 0 should make it non-blocking now
                // since poll() already told us data is ready.
                std::string arduinoResponse = arduinoPort.readLine(0);

                if (!arduinoResponse.empty())
                {
                    std::cout << "Arduino: " << arduinoResponse << std::endl;
                    std::cout.flush();
                }
                // readLine might return empty even if poll reported data,
                // e.g., if only partial data arrived without a newline yet.
                // This is generally fine, the rest will be caught next time.
            }
            else if (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                // Error condition on serial port
                std::cerr << "Error condition on serial port " << port_name << ". Exiting." << std::endl;
                running = false; // Exit loop on serial error
            }
        }
        // Optional small delay to prevent high CPU usage if poll timeout is very short or 0
        // usleep(10000); // e.g., 10ms delay
    } // end while loop

    // Clean up
    std::cout << "Exiting program." << std::endl;
    if (arduinoPort.isOpen())
    {
        std::cout << "Sending final STOP command." << std::endl;
        arduinoPort.writeString("STOP\n");
        usleep(100000); // Give Arduino time to process stop
    }

    // SerialPort destructor will handle closing the port
    std::cout << "Serial port will be closed by destructor." << std::endl;
    return 0;
}