#include "solo_usb_controller/solo_tui.hpp"
#include <iostream>
#include <cstdlib>
#include <csignal>
#include <atomic>

// Global flag for signal handling
std::atomic<bool> g_shutdown_requested(false);

void signalHandler(int signum) {
    g_shutdown_requested = true;
}

int main(int argc, char** argv) {
    // Set up signal handlers for clean exit
    std::signal(SIGINT, signalHandler);   // Ctrl+C
    std::signal(SIGTERM, signalHandler);  // kill command
    
    std::string device = "/dev/solo_mc_1";
    
    // Parse command line arguments
    if (argc > 1) {
        device = argv[1];
    }
    
    try {
        solo::SoloTUI tui(device);
        tui.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}




