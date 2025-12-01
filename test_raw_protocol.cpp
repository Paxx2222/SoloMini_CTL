#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/select.h>
#include <iomanip>

int main() {
    const char* device = "/dev/solo_mc_1";
    
    // Open device
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open: " << strerror(errno) << std::endl;
        return 1;
    }
    
    // Configure serial
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    
    // Flush
    tcflush(fd, TCIOFLUSH);
    usleep(100000); // 100ms
    
    // Send READ_BUS_VOLTAGE (0x86)
    // Frame: [0xFF][0xFF][ADDR=0][CMD=0x86][0x00][0x00][0x00][0x00][CRC=0x00][0xFE]
    unsigned char cmd[] = {0xFF, 0xFF, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE};
    
    std::cout << "Sending: ";
    for (int i = 0; i < 10; i++) {
        printf("%02X ", cmd[i]);
    }
    std::cout << std::endl;
    
    ssize_t written = write(fd, cmd, 10);
    std::cout << "Wrote " << written << " bytes" << std::endl;
    
    // Wait a bit
    usleep(50000); // 50ms
    
    // Try to read response with select
    unsigned char response[10];
    
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200ms timeout
    
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    
    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    
    if (ret < 0) {
        std::cout << "select() failed" << std::endl;
    } else if (ret == 0) {
        std::cout << "Timeout - no data available" << std::endl;
    } else {
        ssize_t n = read(fd, response, 10);
        std::cout << "Read " << n << " bytes: ";
        for (ssize_t i = 0; i < n; i++) {
            printf("%02X ", response[i]);
        }
        std::cout << std::endl;
    }
    
    close(fd);
    return 0;
}




