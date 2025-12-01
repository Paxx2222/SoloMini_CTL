#include "solo_usb_controller/solo_tui.hpp"
#include <iostream>
#include <cstdlib>

int main(int argc, char** argv) {
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




