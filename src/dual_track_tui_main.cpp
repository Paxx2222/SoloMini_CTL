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
              << "  -h, --help           Show this help message\n"
              << "\n"
              << "Examples:\n"
              << "  " << program << "\n"
              << "  " << program << " -l /dev/ttyACM0 -r /dev/ttyACM1\n"
              << "  " << program << " --left /dev/solo_left --right /dev/solo_right\n"
              << "\n"
              << "Controls:\n"
              << "  W/S or UP/DOWN    - Forward/Reverse\n"
              << "  A/D or LEFT/RIGHT - Turn left/right\n"
              << "  Q/E               - Rotate in place\n"
              << "  M                 - Toggle motor enable\n"
              << "  SPACE             - Emergency stop\n"
              << "  ?                 - Show help overlay\n"
              << "  ESC               - Quit\n";
}

int main(int argc, char** argv) {
    // Set up signal handlers for clean exit
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Default configuration
    solo::DualTrackDriver::Config config;
    
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
        else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            std::cerr << "Use --help for usage information.\n";
            return EXIT_FAILURE;
        }
    }
    
    std::cout << "Dual Track Controller - Stage 2\n";
    std::cout << "Left motor:  " << config.left_device << (config.invert_left ? " (inverted)" : "") << "\n";
    std::cout << "Right motor: " << config.right_device << (config.invert_right ? " (inverted)" : "") << "\n";
    std::cout << "\nStarting TUI...\n";
    
    try {
        solo::DualTrackTUI tui(config);
        tui.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}
