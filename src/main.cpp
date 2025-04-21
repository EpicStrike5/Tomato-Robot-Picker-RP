#include "SerialPort.hpp" // Include the library header
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h> // For usleep (consider std::this_thread::sleep_for)

int main()
{
    const char *port_name = "/dev/ttyACM1"; // CHECK YOURS! Might be /dev/ttyUSB0 etc.
    const int baud_rate = 115200;

    SerialPort arduinoPort; // Create a SerialPort object

    // Open and configure the port
    if (!arduinoPort.openPort(port_name, baud_rate))
    {
        std::cerr << "Failed to open serial port." << std::endl;
        return 1;
    }

    std::cout << "RPi Controller Started. Using SerialPort library." << std::endl;
    std::cout << "Enter commands (e.g., 'ElevatorTO:100,0' or 'STOP') or 'exit'." << std::endl;

    std::string userInput;

    // Give Arduino some time to boot/initialize after opening serial, if needed
    usleep(2000000);       // 2 seconds delay - Adjust as necessary
    arduinoPort.flushIO(); // Flush any garbage data after delay

    while (true)
    {
        // --- Check for incoming Arduino messages (using readLine) ---
        // readLine will wait up to 10ms (default timeout) for a full line.
        // If no full line arrives, it returns empty.
        std::string arduinoResponse = arduinoPort.readLine(10); // Short timeout (10ms) for non-blocking feel

        if (!arduinoResponse.empty())
        {
            std::cout << "Arduino: " << arduinoResponse << std::endl;
            std::cout.flush();
        }
        else
        {
            // Optional: check if port is still open if readLine returns empty repeatedly
            // Might indicate a disconnected device.
            // if (!arduinoPort.isOpen()) { /* handle error */ }
        }

        // --- Check for User Input ---
        // Simple blocking read for user input.
        // For more responsive non-blocking input AND serial reading,
        // you'd use poll() or select() on BOTH stdin (fd 0) and the serial port (arduinoPort.getFD() - needs getter).
        std::cout << "> ";
        std::cout.flush(); // Make sure prompt appears

        if (!std::getline(std::cin, userInput))
        {
            if (std::cin.eof())
            { // Check for Ctrl+D
                std::cout << "EOF detected, exiting." << std::endl;
            }
            else
            {
                std::cerr << "Error reading stdin." << std::endl;
            }
            break; // Exit loop on input error or EOF
        }

        if (userInput == "exit")
        {
            break;
        }

        // --- Process and Send Command ---
        bool command_sent = false;
        if (!userInput.empty())
        {
            // Add the newline terminator required by the original Arduino code
            std::string commandToSend = userInput + "\n";
            command_sent = arduinoPort.writeString(commandToSend);

            if (!command_sent)
            {
                std::cerr << "Failed to send command: " << userInput << std::endl;
                // Optionally, check if the port is still open, try to reopen, or exit
                if (!arduinoPort.isOpen())
                {
                    std::cerr << "Serial port appears closed. Exiting." << std::endl;
                    break;
                }
            }
            else
            {
                // Optional small delay after sending, if the Arduino needs it
                // usleep(50000); // 50ms
            }
        }

    } // end while loop

    // Clean up (SerialPort destructor handles closing the port automatically)
    std::cout << "Exiting. Sending final STOP command." << std::endl;
    if (arduinoPort.isOpen())
    {
        arduinoPort.writeString("STOP\n");
        usleep(100000); // Give Arduino time to process stop
        // arduinoPort.closePort(); // Explicit close, or let destructor handle it
    }

    std::cout << "Serial port closed by destructor." << std::endl;
    return 0;
}