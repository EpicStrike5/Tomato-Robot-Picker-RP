#include "SerialPort.h" // Your serial port library header
#include <iostream>
#include <string>
#include <thread>     // For std::thread
#include <atomic>     // For std::atomic<bool>
#include <chrono>     // For std::chrono::milliseconds, std::chrono::seconds
#include <vector>     // For std::vector
#include <stdexcept>  // For std::exception
#include <csignal>    // For signal handling (Ctrl+C)
#include <filesystem> // For iterating through files (Requires C++17)
#include <memory>     // For std::shared_ptr
#include <algorithm>  // For std::transform (used in run_ai_test_on_folder)

// --- Include headers for YOLO detection (New Header) ---
#include <opencv2/opencv.hpp> // Still need core OpenCV functionalities
#include "YOLO12.hpp"         // Include the new detector header

// --- Configuration Constants ---
const std::string SERIAL_PORT = "/dev/ttyACM0"; // CHECK THIS!
const speed_t SERIAL_BAUD = B115200;
// --- ADJUST PATHS FOR YOUR TOMATO MODEL ---
const std::string TOMATO_MODEL_PATH = "AIModel/TomatoDetectBTV1.onnx"; // ADJUST PATH to your .onnx file
const std::string TOMATO_CLASS_NAMES_PATH = "AIModel/tomato.names";    // ADJUST PATH to your .names file
const std::string TEST_IMAGE_FOLDER_PATH = "AIModel/Test_images/";     // ADJUST PATH
const std::string OUTPUT_IMAGE_FOLDER_PATH = "AIModel/Output_images/"; // ** NEW: Path to save results **

// --- ** NEW: Adjustable Thresholds for Experimentation ** ---
const float DETECT_CONF_THRESHOLD = 0.4f; // Default 0.4f - Try lowering (e.g., 0.25f)
const float DETECT_IOU_THRESHOLD = 0.45f; // Default 0.45f - Try adjusting slightly if needed

const bool USE_GPU_IF_AVAILABLE = false; // Set to false for Raspberry Pi CPU usually

// Global atomic flag to signal threads to stop
std::atomic<bool> keep_running(true);

// Global detector object (using new class)
std::shared_ptr<YOLO12Detector> detector = nullptr;
// NOTE: classNames and classColors are now managed internally by YOLO12Detector via YOLO12.hpp

// Signal handler function for SIGINT (Ctrl+C)
void signalHandler(int signum)
{
    std::cerr << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    keep_running.store(false); // Signal threads to stop
}

// --- Serial Reading Thread (Unchanged from your previous code) ---
void read_from_arduino(SerialPort &serial_port)
{
    std::string line_buffer;
    char read_buffer[256];
    std::cout << "[Reader thread started - using non-blocking reads]" << std::endl;

    while (keep_running.load())
    {
        try
        {
            if (!serial_port.isOpen())
            {
                if (keep_running.load())
                {
                    std::cerr << "[Reader] Serial port appears closed. Exiting thread." << std::endl;
                }
                break;
            }

            ssize_t bytes_read = serial_port.readBytes(reinterpret_cast<uint8_t *>(read_buffer), sizeof(read_buffer) - 1);

            if (bytes_read > 0)
            {
                read_buffer[bytes_read] = '\0';
                line_buffer.append(read_buffer, bytes_read);

                size_t newline_pos;
                while ((newline_pos = line_buffer.find('\n')) != std::string::npos)
                {
                    std::string complete_line = line_buffer.substr(0, newline_pos + 1);
                    line_buffer.erase(0, newline_pos + 1);

                    if (!complete_line.empty() && complete_line.back() == '\n')
                    {
                        complete_line.pop_back();
                        if (!complete_line.empty() && complete_line.back() == '\r')
                        {
                            complete_line.pop_back();
                        }
                    }
                    // Ensure prompt appears correctly after Arduino message
                    std::cout << "\nArduino: " << complete_line << "\n> " << std::flush;
                }
            }
            else if (bytes_read == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        catch (const std::system_error &e)
        {
            if (keep_running.load())
            {
                std::cerr << "\n[Reader] Serial read error: " << e.what() << " (code: " << e.code() << "). Exiting thread." << std::endl;
                keep_running.store(false);
            }
            break;
        }
        catch (const std::exception &e)
        {
            if (keep_running.load())
            {
                std::cerr << "\n[Reader] Exception: " << e.what() << ". Exiting thread." << std::endl;
                keep_running.store(false);
            }
            break;
        }
    }
    std::cout << "[Reader thread finished]" << std::endl;
}

// Function to trim leading/trailing whitespace (Unchanged)
std::string trim(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t\n\r");
    if (std::string::npos == first)
    {
        return "";
    }
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, (last - first + 1));
}

// --- Function to run AI detection on images in a folder (Updated) ---
void run_ai_test_on_folder(const std::string &folderPath)
{
    std::cout << "\n[AI Test] Starting test on folder: " << folderPath << std::endl;
    if (!detector)
    { // Check if the global detector object is initialized
        std::cerr << "[AI Test] Error: Detector not initialized." << std::endl;
        std::cout << "> " << std::flush; // Re-display prompt
        return;
    }

    // ** NEW: Ensure output directory exists **
    if (!std::filesystem::exists(OUTPUT_IMAGE_FOLDER_PATH))
    {
        try
        {
            std::filesystem::create_directories(OUTPUT_IMAGE_FOLDER_PATH);
            std::cout << "[AI Test] Created output directory: " << OUTPUT_IMAGE_FOLDER_PATH << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[AI Test] Error creating output directory " << OUTPUT_IMAGE_FOLDER_PATH << ": " << e.what() << std::endl;
            std::cout << "> " << std::flush;
            return; // Stop if we can't create output dir
        }
    }

    int image_count = 0;
    try
    {
        // Requires C++17 filesystem library
        for (const auto &entry : std::filesystem::directory_iterator(folderPath))
        {
            if (entry.is_regular_file())
            {
                std::filesystem::path inputPath = entry.path(); // Use path object
                std::string imagePathStr = inputPath.string();

                // Basic check for image extensions
                std::string ext = inputPath.extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower); // Convert extension to lower case
                if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp")
                {
                    std::cout << "[AI Test] Processing image: " << imagePathStr << std::endl;
                    cv::Mat image = cv::imread(imagePathStr);
                    if (image.empty())
                    {
                        std::cerr << "[AI Test] Warning: Could not read image: " << imagePathStr << std::endl;
                        continue;
                    }
                    image_count++;

                    try
                    {
                        auto start = std::chrono::high_resolution_clock::now();
                        // ** Use the adjustable thresholds **
                        std::vector<Detection> result = detector->detect(image, DETECT_CONF_THRESHOLD, DETECT_IOU_THRESHOLD);

                        auto stop = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                        std::cout << "[AI Test] Detection took: " << duration.count() << " ms. Found " << result.size() << " objects." << std::endl;

                        // Draw bounding boxes on the image
                        detector->drawBoundingBox(image, result);

                        // ** REMOVED Display part that causes crash **
                        // cv::imshow("AI Test Result - Press any key", image);
                        // cv::waitKey(0);

                        // ** NEW: Save the image with detections **
                        std::string outputFilename = inputPath.stem().string() + "_detected" + inputPath.extension().string();
                        std::string outputPathStr = OUTPUT_IMAGE_FOLDER_PATH + outputFilename;

                        if (cv::imwrite(outputPathStr, image))
                        {
                            std::cout << "[AI Test] Saved result to: " << outputPathStr << std::endl;
                        }
                        else
                        {
                            std::cerr << "[AI Test] Error: Could not save result image to: " << outputPathStr << std::endl;
                        }
                    }
                    catch (const std::exception &e)
                    {
                        std::cerr << "[AI Test] Error during detection/saving on " << imagePathStr << ": " << e.what() << std::endl;
                    }
                }
            }
        }
        // ** REMOVED as no windows are created **
        // cv::destroyAllWindows();
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        std::cerr << "[AI Test] Filesystem error accessing folder " << folderPath << ": " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[AI Test] General error during folder processing: " << e.what() << std::endl;
    }

    if (image_count == 0)
    {
        std::cout << "[AI Test] No compatible image files (.jpg, .png, .bmp) found in " << folderPath << std::endl;
    }
    else
    {
        std::cout << "[AI Test] Finished processing " << image_count << " images." << std::endl;
    }
    std::cout << "> " << std::flush; // Re-display prompt
}

// Main function
int main()
{
    signal(SIGINT, signalHandler); // Register signal handler

    try
    {
        // --- Initialize Serial Port ---
        SerialPort serial_port(SERIAL_PORT, SERIAL_BAUD);
        std::cout << "Serial port '" << SERIAL_PORT << "' opened. Arduino should be initialized." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Allow Arduino time to boot

        // --- Initialize YOLO Detector (Using new YOLO12Detector class) ---
        std::cout << "Initializing YOLOv12 detector..." << std::endl;
        try
        {
            // Create the detector object using the model and class names paths
            detector = std::make_shared<YOLO12Detector>(TOMATO_MODEL_PATH, TOMATO_CLASS_NAMES_PATH, USE_GPU_IF_AVAILABLE);
            std::cout << "YOLOv12 detector initialized successfully." << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Fatal Error: Failed to initialize YOLOv12 detector: " << e.what() << std::endl;
            std::cerr << "Ensure model file '" << TOMATO_MODEL_PATH << "' and names file '" << TOMATO_CLASS_NAMES_PATH << "' exist and are valid." << std::endl;
            std::cerr << "Also check ONNX Runtime and OpenCV installations." << std::endl;
            std::cerr << "AI features will be disabled." << std::endl;
            // Decide if you want to exit or continue without AI
            // return 1; // Exit if AI is critical
        }

        // --- Start Threads ---
        std::thread reader_thread(read_from_arduino, std::ref(serial_port));

        // --- Main loop: Handle user input ---
        std::cout << "Enter commands (e.g., 'MoveTO:180,108', 'TestAI', 'AINow', 'exit')." << std::endl;
        std::string user_input;
        while (keep_running.load())
        {
            std::cout << "> " << std::flush; // Display prompt

            if (!std::getline(std::cin, user_input))
            {
                if (std::cin.eof())
                {
                    std::cout << "\n[Input] End of input detected (Ctrl+D)." << std::endl;
                }
                else if (keep_running.load())
                {
                    std::cerr << "\n[Input] Error reading standard input." << std::endl;
                }
                keep_running.store(false);
                break;
            }

            if (!keep_running.load())
                break; // Check if Ctrl+C was pressed during getline

            std::string trimmed_input = trim(user_input);
            if (trimmed_input.empty())
                continue; // Ignore empty lines

            // --- Command Handling ---
            if (trimmed_input == "exit" || trimmed_input == "quit")
            {
                keep_running.store(false);
                break;
            }
            else if (trimmed_input == "TestAI")
            {
                // Call the AI testing function
                run_ai_test_on_folder(TEST_IMAGE_FOLDER_PATH);
                // Don't send this command to Arduino
            }
            else if (trimmed_input == "AINow")
            {
                // Placeholder for future camera-based detection
                std::cout << "\n[AI] 'AINow' command received. (Camera detection not implemented yet)..." << std::endl;
                if (!detector)
                {
                    std::cerr << "[AI] Cannot run AINow: Detector not initialized." << std::endl;
                }
                else
                {
                    // --- Future: Add camera capture and detection logic here ---
                    std::cout << "[AI] (Placeholder) Would run detection on camera feed now." << std::endl;
                    // Example: run_ai_on_camera_frame(); // Need to implement this function
                }
                std::cout << "> " << std::flush; // Re-display prompt
                                                 // Decide if you need to send anything to Arduino for this command
            }
            else
            {
                // --- Default: Send command to Arduino ---
                if (trimmed_input.back() != '\n')
                {
                    trimmed_input += '\n'; // Ensure newline termination for Arduino parser
                }
                try
                {
                    if (!serial_port.isOpen())
                    {
                        std::cerr << "[Input] Serial port is not open. Cannot send command." << std::endl;
                        keep_running.store(false);
                        break;
                    }
                    serial_port.writeString(trimmed_input);
                    // Optional echo: std::cout << "\n[Pi Sent: " << trimmed_input << "]" << std::flush;
                }
                catch (const std::system_error &e)
                {
                    std::cerr << "\n[Input] Serial write error: " << e.what() << " (code: " << e.code() << ")" << std::endl;
                    keep_running.store(false);
                    break;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "\n[Input] Exception during write: " << e.what() << std::endl;
                    keep_running.store(false);
                    break;
                }
            }
        } // End while(keep_running)

        // --- Cleanup ---
        std::cout << "\n[Main] Shutting down..." << std::endl;
        keep_running.store(false); // Ensure flag is false

        // Close OpenCV windows just in case they were left open by TestAI interrupt
        cv::destroyAllWindows();

        if (reader_thread.joinable())
        {
            reader_thread.join();
        }
        std::cout << "[Main] Reader thread joined." << std::endl;
        // SerialPort destructor closes the port automatically.
        // Detector shared_ptr cleans up automatically.
    }
    catch (const std::system_error &e)
    {
        std::cerr << "Fatal Error: Serial Port Initialization Failed: " << e.what() << " (code: " << e.code() << ")" << std::endl;
        // Add troubleshooting tips here if desired
        std::cerr << "Troubleshooting tips:" << std::endl;
        std::cerr << " - Is the port name '" << SERIAL_PORT << "' correct? (Check dmesg, ls /dev/ttyACM*)" << std::endl;
        // ... (rest of tips)
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "An unexpected error occurred during setup or runtime: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "[Main] Program finished cleanly." << std::endl;
    return 0;
}