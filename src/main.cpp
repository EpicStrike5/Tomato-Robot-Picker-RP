#include "SerialPort.h" // Your serial port library header
#include <iostream>
#include <string>
#include <sstream> // ** NEW: For formatting output string **
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <stdexcept>
#include <csignal>
#include <filesystem>
#include <memory>
#include <algorithm>
#include <iomanip> // ** NEW: For std::setprecision **

// --- Include headers for YOLO detection ---
#include <opencv2/opencv.hpp>
#include "YOLO12.hpp"

// --- ** NEW: Include headers for RealSense ** ---
#include <librealsense2/rs.hpp>
#include "cv-helpers.hpp" // Make sure this is in your include path

// --- Configuration Constants ---
const std::string SERIAL_PORT = "/dev/ttyACM0";
const speed_t SERIAL_BAUD = B115200;
const std::string TOMATO_MODEL_PATH = "AIModel/TomatoDetectFastBTV2.onnx";
const std::string TOMATO_CLASS_NAMES_PATH = "AIModel/tomato.names";
const std::string TEST_IMAGE_FOLDER_PATH = "AIModel/Test_images/";
const std::string OUTPUT_IMAGE_FOLDER_PATH = "AIModel/Output_images/";
const std::string REALSENSE_OUTPUT_FOLDER_PATH = "AIModel/Realsense/"; // ** NEW: Output for AINow **

// --- Adjustable Thresholds ---
const float DETECT_CONF_THRESHOLD = 0.40f; // Adjusted based on HUB results
const float DETECT_IOU_THRESHOLD = 0.60f;  // Adjusted based on HUB results

const bool USE_GPU_IF_AVAILABLE = false;

// --- ** NEW: RealSense Constants ** ---
const int RS_WIDTH = 640;
const int RS_HEIGHT = 480;
const int RS_FPS = 30;

// Global atomic flag to signal threads to stop
std::atomic<bool> keep_running(true);

// Global detector object
std::shared_ptr<YOLO12Detector> detector = nullptr;

// --- ** NEW: Global RealSense Objects ** ---
rs2::pipeline rs_pipe;                       // Declare pipeline object
rs2::config cfg;                             // Declare configuration object
rs2::align align_to_color(RS2_STREAM_COLOR); // Declare alignment object
rs2_intrinsics depth_intrinsics;             // Store depth sensor intrinsics
bool realsense_initialized = false;          // Flag to track initialization

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

// --- ** NEW: Function to get timestamp for filenames ** ---
std::string get_timestamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss.str();
}

void run_ai_on_camera()
{
    if (!realsense_initialized)
    {
        std::cerr << "\n[AI] Error: RealSense camera not initialized." << std::endl;
        std::cout << "> " << std::flush;
        return;
    }
    if (!detector)
    {
        std::cerr << "\n[AI] Error: YOLO Detector not initialized." << std::endl;
        std::cout << "> " << std::flush;
        return;
    }

    std::cout << "\n[AI] Capturing frame from RealSense..." << std::endl;

    try
    {
        rs2::frameset frames = rs_pipe.wait_for_frames();     // Wait for a coherent set of frames
        auto aligned_frames = align_to_color.process(frames); // Align depth to color viewport

        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();

        if (!color_frame || !depth_frame)
        {
            std::cerr << "[AI] Error: Could not get valid color/depth frame." << std::endl;
            std::cout << "> " << std::flush;
            return;
        }

        // Convert color frame to OpenCV Mat
        cv::Mat color_mat = frame_to_mat(color_frame); // Using cv-helpers.hpp function

        // --- Run Detection ---
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Detection> detections = detector->detect(color_mat, DETECT_CONF_THRESHOLD, DETECT_IOU_THRESHOLD);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[AI] Detection took: " << duration.count() << " ms. Found " << detections.size() << " objects." << std::endl;

        // --- Process Detections for Coordinates ---
        std::stringstream coord_ss; // To build coordinate string
        coord_ss << "Detections: ";
        int detection_count = 0;

        for (const auto &det : detections)
        {
            if (det.classId < 0)
                continue; // Should not happen with YOLO12.hpp but good check

            // Get BBox center
            int cx = det.box.x + det.box.width / 2;
            int cy = det.box.y + det.box.height / 2;

            // Clamp coordinates to be within frame dimensions before querying depth
            cx = std::max(0, std::min(cx, RS_WIDTH - 1));
            cy = std::max(0, std::min(cy, RS_HEIGHT - 1));

            // Get depth value at center pixel (in meters)
            float depth_m = depth_frame.get_distance(cx, cy);

            if (depth_m > 0.01)
            { // Check for valid depth (ignore 0 or very close)
                // Deproject pixel to 3D point
                float pixel[2] = {(float)cx, (float)cy};
                float point[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z
                // Use the stored depth intrinsics
                rs2_deproject_pixel_to_point(point, &depth_intrinsics, pixel, depth_m);

                // Append to string (format: ClassName: [X, Y, Z], )
                // Assuming detector->classNames exists or get class name via ID
                // Note: YOLO12.hpp keeps classNames private, so we'll just use ClassID for now.
                // If you need names, you'd need to modify YOLO12.hpp to expose classNames or reload them here.
                if (detection_count > 0)
                    coord_ss << ", ";
                coord_ss << "ID" << det.classId << ": [" << std::fixed << std::setprecision(3) << point[0] << ", "
                         << point[1] << ", " << point[2] << "]";
                detection_count++;
            }
            else
            {
                if (detection_count > 0)
                    coord_ss << ", ";
                coord_ss << "ID" << det.classId << ": [depth_invalid]";
                detection_count++;
                std::cout << "[AI] Warning: Invalid depth (0) for detection ID " << det.classId << " at pixel (" << cx << ", " << cy << ")" << std::endl;
            }
        }

        // Print coordinate string
        std::cout << coord_ss.str() << std::endl;

        // --- Save Image ---
        // Ensure output directory exists
        if (!std::filesystem::exists(REALSENSE_OUTPUT_FOLDER_PATH))
        {
            try
            {
                std::filesystem::create_directories(REALSENSE_OUTPUT_FOLDER_PATH);
                std::cout << "[AI] Created output directory: " << REALSENSE_OUTPUT_FOLDER_PATH << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cerr << "[AI] Error creating output directory " << REALSENSE_OUTPUT_FOLDER_PATH << ": " << e.what() << std::endl;
                // Continue without saving if directory creation fails
            }
        }

        // Draw boxes on the image
        detector->drawBoundingBox(color_mat, detections);

        // Generate filename and save
        std::string timestamp = get_timestamp();
        std::string output_filename = REALSENSE_OUTPUT_FOLDER_PATH + "capture_" + timestamp + "_detected.jpg";
        if (cv::imwrite(output_filename, color_mat))
        {
            std::cout << "[AI] Saved detection result to: " << output_filename << std::endl;
        }
        else
        {
            std::cerr << "[AI] Error: Failed to save image to " << output_filename << std::endl;
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr << "[AI] RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[AI] Error during camera processing: " << e.what() << std::endl;
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

        std::cout << "Initializing RealSense D435..." << std::endl;
        try
        {
            // Configure streams
            cfg.enable_stream(RS2_STREAM_DEPTH, RS_WIDTH, RS_HEIGHT, RS2_FORMAT_Z16, RS_FPS);
            cfg.enable_stream(RS2_STREAM_COLOR, RS_WIDTH, RS_HEIGHT, RS2_FORMAT_BGR8, RS_FPS); // Request BGR for direct OpenCV use

            // Start pipeline
            rs2::pipeline_profile profile = rs_pipe.start(cfg);

            // Get depth sensor intrinsics (important for deprojection)
            rs2::stream_profile depth_stream_profile = profile.get_stream(RS2_STREAM_DEPTH);
            depth_intrinsics = depth_stream_profile.as<rs2::video_stream_profile>().get_intrinsics();
            std::cout << "Depth Intrinsics: fx=" << depth_intrinsics.fx << ", fy=" << depth_intrinsics.fy
                      << ", ppx=" << depth_intrinsics.ppx << ", ppy=" << depth_intrinsics.ppy << std::endl;

            // Allow auto-exposure to stabilize
            std::cout << "Waiting for auto-exposure..." << std::endl;
            for (int i = 0; i < 30; ++i)
                rs_pipe.wait_for_frames();
            std::cout << "RealSense initialized successfully." << std::endl;
            realsense_initialized = true;
        }
        catch (const rs2::error &e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            std::cerr << "RealSense features will be disabled." << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error initializing RealSense: " << e.what() << std::endl;
            std::cerr << "RealSense features will be disabled." << std::endl;
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
                // ** Call the new camera function **
                run_ai_on_camera();
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
        keep_running.store(false);

        // ** NEW: Stop RealSense pipeline **
        if (realsense_initialized)
        {
            std::cout << "[Main] Stopping RealSense pipeline..." << std::endl;
            rs_pipe.stop();
        }

        if (reader_thread.joinable())
        {
            reader_thread.join();
        }
        std::cout << "[Main] Reader thread joined." << std::endl;
    }
    catch (const std::system_error &e)
    { /* ... Serial Error handling ... */
    }
    catch (const std::exception &e)
    { /* ... General Error handling ... */
    }

    std::cout << "[Main] Program finished cleanly." << std::endl;
    return 0;
}