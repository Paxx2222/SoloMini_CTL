#include "solo_usb_controller/dual_track_tui.hpp"
#include <iostream>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <cstring>

// Global flag for signal handling
std::atomic<bool> g_shutdown_requested(false);

void signalHandler(int /*signum*/) {
    g_shutdown_requested = true;
}

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [OPTIONS]\n"
              << "\n"
              << "Dual Track Controller TUI - Stage 2\n"
              << "Controls two SOLO motor controllers for tracked vehicle operation.\n"
              << "\n"
              << "Options:\n"
              << "  -l, --left DEVICE    Left motor device (default: /dev/solo_left)\n"
              << "  -r, --right DEVICE   Right motor device (default: /dev/solo_right)\n"
              << "  --invert-left        Invert left motor direction\n"
              << "  --no-invert-right    Don't invert right motor (default: inverted)\n"
              << "  --ros                Enable ROS2 topic control (/ibus/ch2, /ibus/ch1)\n"
              << "  -h, --help           Show this help message\n"
              << "\n"
              << "Examples:\n"
              << "  " << program << "\n"
              << "  " << program << " -l /dev/ttyACM0 -r /dev/ttyACM1\n"
              << "  " << program << " --ros\n"
              << "  " << program << " --left /dev/solo_left --right /dev/solo_right --ros\n"
              << "\n"
              << "Controls:\n"
              << "  W/S or UP/DOWN    - Forward/Reverse (disabled when ROS2 active)\n"
              << "  A/D or LEFT/RIGHT - Turn left/right (disabled when ROS2 active)\n"
              << "  Q/E               - Rotate in place (disabled when ROS2 active)\n"
              << "  M                 - Toggle motor enable\n"
              << "  SPACE             - Emergency stop\n"
              << "  ?                 - Show help overlay\n"
              << "  ESC               - Quit\n"
              << "\n"
              << "ROS2 Topics (when --ros enabled):\n"
              << "  /ibus/ch2         - Speed control (std_msgs/Float64: -1=reverse, 1=forward)\n"
              << "  /ibus/ch1         - Steering control (std_msgs/Float64: speed difference)\n";
}

int main(int argc, char** argv) {
    // Set up signal handlers for clean exit
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Default configuration
    solo::DualTrackDriver::Config config;
    bool enable_ros = false;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return EXIT_SUCCESS;
        }
        else if ((strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--left") == 0) && i + 1 < argc) {
            config.left_device = argv[++i];
        }
        else if ((strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--right") == 0) && i + 1 < argc) {
            config.right_device = argv[++i];
        }
        else if (strcmp(argv[i], "--invert-left") == 0) {
            config.invert_left = true;
        }
        else if (strcmp(argv[i], "--no-invert-right") == 0) {
            config.invert_right = false;
        }
        else if (strcmp(argv[i], "--ros") == 0) {
            enable_ros = true;
        }
        else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            std::cerr << "Use --help for usage information.\n";
            return EXIT_FAILURE;
        }
    }
    
    std::cout << "Dual Track Controller - Stage 2\n";
    std::cout << "Left motor:  " << config.left_device << (config.invert_left ? " (inverted)" : "") << "\n";
    std::cout << "Right motor: " << config.right_device << (config.invert_right ? " (inverted)" : "") << "\n";
    if (enable_ros) {
        std::cout << "ROS2 topics: ENABLED (/ibus/ch2, /ibus/ch1)\n";
    }
    std::cout << "\nStarting TUI...\n";
    
    try {
        solo::DualTrackTUI tui(config, enable_ros);
        tui.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}
