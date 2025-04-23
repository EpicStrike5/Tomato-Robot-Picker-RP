#include "SerialPort.h" // Your serial port library header
#include <iostream>
#include <string>
#include <thread>    // For std::thread
#include <atomic>    // For std::atomic<bool>
#include <chrono>    // For std::chrono::milliseconds, std::chrono::seconds
#include <vector>    // For read buffer (though char array used here)
#include <stdexcept> // For std::exception
#include <csignal>   // For signal handling (Ctrl+C)
#include <string>    // Ensure std::string is available
#include <unistd.h>  // Provides STDIN_FILENO if needed
#include <poll.h>    // For poll() if needed (not used in this example

// Global atomic flag to signal threads to stop
std::atomic<bool> keep_running(true);

// Signal handler function for SIGINT (Ctrl+C)
void signalHandler(int signum)
{
    // Using cerr for signal handler output is generally safer
    std::cerr << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    keep_running.store(false); // Signal threads to stop
}

// Function executed by the reading thread - Using non-blocking readBytes + sleep
void read_from_arduino(SerialPort &serial_port)
{                            // Pass SerialPort object by reference
    std::string line_buffer; // Persistent buffer to accumulate data across reads
    char read_buffer[256];   // Temporary buffer for each readBytes() call

    std::cout << "[Reader thread started - using non-blocking reads]" << std::endl;

    while (keep_running.load())
    {
        try
        {
            // Check if the port is still open before attempting read
            if (!serial_port.isOpen())
            {
                if (keep_running.load())
                { // Avoid message during normal shutdown
                    std::cerr << "[Reader] Serial port appears closed. Exiting thread." << std::endl;
                }
                break; // Exit thread loop
            }

            // Call the non-blocking readBytes.
            // Returns >0 if data was read, 0 if no data available now, throws on error.
            ssize_t bytes_read = serial_port.readBytes(reinterpret_cast<uint8_t *>(read_buffer), sizeof(read_buffer) - 1); // Leave space for null terminator

            if (bytes_read > 0)
            {
                // Data received, append to the line buffer
                read_buffer[bytes_read] = '\0'; // Null-terminate for safety if needed elsewhere, append handles length
                line_buffer.append(read_buffer, bytes_read);

                // Process all complete lines (`\n` terminated) found in the buffer
                size_t newline_pos;
                while ((newline_pos = line_buffer.find('\n')) != std::string::npos)
                {
                    // Extract the complete line (up to and including the newline)
                    std::string complete_line = line_buffer.substr(0, newline_pos + 1);

                    // Remove the processed line from the front of the buffer
                    line_buffer.erase(0, newline_pos + 1);

                    // Prepare for printing: remove trailing \n and potential \r
                    if (!complete_line.empty() && complete_line.back() == '\n')
                    {
                        complete_line.pop_back(); // Remove \n
                        if (!complete_line.empty() && complete_line.back() == '\r')
                        {
                            complete_line.pop_back(); // Remove \r
                        }
                    }

                    // Print the complete line
                    // Add a newline BEFORE printing "Arduino:" to help separate from user prompt
                    std::cout << "\nArduino: " << complete_line << std::flush;
                    // Consider re-printing prompt "> " here if needed:
                    // std::cout << "\nArduino: " << complete_line << "\n> " << std::flush;
                }
                // Any data left in line_buffer is an incomplete line; it stays for the next read.
            }
            else if (bytes_read == 0)
            {
                // No data available right now. This is expected in non-blocking mode.
                // Sleep briefly to prevent the loop from consuming 100% CPU.
                std::this_thread::sleep_for(std::chrono::milliseconds(20)); // e.g., 20ms sleep
            }
            // Negative bytes_read indicates an error, which readBytes should have thrown
            // unless it was EAGAIN/EWOULDBLOCK (which returns 0 now).
        }
        catch (const std::system_error &e)
        {
            if (keep_running.load())
            { // Report error only if not shutting down
                std::cerr << "\n[Reader] Serial read error: " << e.what() << " (code: " << e.code() << "). Exiting thread." << std::endl;
                keep_running.store(false); // Signal main thread to stop
            }
            break; // Exit the loop on error
        }
        catch (const std::exception &e)
        { // Catch other potential exceptions (e.g., runtime_error)
            if (keep_running.load())
            {
                std::cerr << "\n[Reader] Exception: " << e.what() << ". Exiting thread." << std::endl;
                keep_running.store(false); // Signal main thread
            }
            break; // Exit the loop on error
        }
    }
    std::cout << "[Reader thread finished]" << std::endl;
}

// Function to trim leading/trailing whitespace
std::string trim(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t\n\r");
    if (std::string::npos == first)
    {
        return ""; // Return empty if string is all whitespace
    }
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, (last - first + 1));
}

// Main function - Handles setup, input thread, writing, and cleanup
int main()
{
    // Register signal handler for SIGINT (Ctrl+C) for graceful shutdown
    signal(SIGINT, signalHandler);

    // --- Configuration ---
    // Update port based on user's ls output or where Arduino appears
    std::string port = "/dev/ttyACM0"; // Or /dev/ttyACM1, /dev/ttyACM2 etc. CHECK THIS!
    speed_t baud = B115200;            // Match Arduino sketch

    try
    {
        // Create and open the serial port (now configured non-blocking inside constructor)
        SerialPort serial_port(port, baud);

        std::cout << "Serial port '" << port << "' opened. Arduino should be initialized." << std::endl;
        std::cout << "Enter commands to send (e.g., 'MoveTO:50.0,10.0', 'ArmTO:100', 'STOP')." << std::endl;
        std::cout << "Type 'exit' or 'quit' or press Ctrl+C to close." << std::endl;

        // Short pause allowing Arduino bootloader/setup to complete after port open
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Launch the reader thread, passing the SerialPort object by reference
        std::thread reader_thread(read_from_arduino, std::ref(serial_port));

        // --- Main thread: Handle user input and writing ---
        std::string user_input;
        while (keep_running.load())
        {
            std::cout << "> " << std::flush; // Display prompt

            // Read a line of input from the user
            if (!std::getline(std::cin, user_input))
            {
                // Handle end-of-file (e.g., Ctrl+D) or input error
                if (std::cin.eof())
                {
                    std::cout << "\n[Input] End of input detected (Ctrl+D)." << std::endl;
                }
                else
                {
                    // Only print error if not triggered by shutdown signal
                    if (keep_running.load())
                    {
                        std::cerr << "\n[Input] Error reading standard input." << std::endl;
                    }
                }
                keep_running.store(false); // Signal shutdown
                break;                     // Exit input loop
            }

            // Check if the program is already signalled to stop (e.g., by Ctrl+C)
            if (!keep_running.load())
            {
                break;
            }

            std::string trimmed_input = trim(user_input);

            if (trimmed_input.empty())
            {
                continue; // Ignore empty lines, re-prompt
            }

            // Check for exit commands
            if (trimmed_input == "exit" || trimmed_input == "quit")
            {
                keep_running.store(false); // Signal threads to stop
                break;                     // Exit input loop
            }

            // Add newline terminator required by Arduino's CommandParser
            if (trimmed_input.back() != '\n')
            {
                trimmed_input += '\n';
            }

            // Send the command via the serial port
            try
            {
                // Check port status before writing
                if (!serial_port.isOpen())
                {
                    std::cerr << "[Input] Serial port is not open. Cannot send command." << std::endl;
                    keep_running.store(false); // Signal shutdown
                    break;
                }
                serial_port.writeString(trimmed_input);
                // Optional echo to confirm sending:
                // std::cout << "\n[Pi Sent: " << trimmed_input << "]" << std::flush;
            }
            catch (const std::system_error &e)
            {
                std::cerr << "\n[Input] Serial write error: " << e.what() << " (code: " << e.code() << ")" << std::endl;
                keep_running.store(false); // Stop on write error
                break;
            }
            catch (const std::exception &e)
            {
                std::cerr << "\n[Input] Exception during write: " << e.what() << std::endl;
                keep_running.store(false); // Stop on write error
                break;
            }
            // Loop continues for next command input
        }

        // --- Cleanup ---
        std::cout << "\n[Main] Shutting down..." << std::endl;
        keep_running.store(false); // Ensure flag is false for reader thread exit check

        // Wait for the reader thread to finish its loop and exit
        if (reader_thread.joinable())
        {
            reader_thread.join();
        }
        std::cout << "[Main] Reader thread joined." << std::endl;

        // SerialPort destructor runs automatically when 'serial_port' goes out of scope,
        // closing the port via closePort().
    }
    catch (const std::system_error &e)
    {
        std::cerr << "Fatal Error: Failed to initialize serial port '" << port << "': "
                  << e.what() << " (errno: " << e.code() << ")" << std::endl;
        // Print troubleshooting tips
        std::cerr << "Troubleshooting tips:" << std::endl;
        std::cerr << " - Is the port name '" << port << "' correct? (Check dmesg, ls /dev/ttyACM*)" << std::endl;
        std::cerr << " - Is the Arduino connected, powered, and running the sketch?" << std::endl;
        std::cerr << " - Does the user '" << (getenv("USER") ? getenv("USER") : "current") << "' belong to the 'dialout' group? (sudo usermod -a -G dialout $USER)" << std::endl;
        std::cerr << " - Is another program (like Arduino IDE Serial Monitor, minicom) using the port?" << std::endl;
        std::cerr << " - Does the baud rate (115200) match the Arduino sketch?" << std::endl;
        return 1; // Indicate error
    }
    catch (const std::exception &e)
    { // Catch other potential setup errors
        std::cerr << "An unexpected error occurred during setup: " << e.what() << std::endl;
        return 1; // Indicate error
    }

    std::cout << "[Main] Program finished cleanly." << std::endl;
    return 0; // Indicate success
}