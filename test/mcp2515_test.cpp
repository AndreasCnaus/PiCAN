#include "../common/mcp2515_ioctl.h"
#include "can_communication.h"

#include <iostream>     // required for standard C++ input/output stream library
#include <stdexcept>    // required for standard exceptions
#include <cstring>      // required for C string handling functions like strerror
#include <cerrno>       // required for errno macro to report error codes
#include <atomic>       // required for atomic variables used in multithreading
#include <mutex>        // required for std::mutex and std::lock_guard to manage concurrent access
#include <sstream>      // required for string stream operations
#include <thread>       // required for using std::thread for multithreading
#include <csignal>      // required for handling POSIX signals
#include <sys/ioctl.h>  // required for ioctl system call for device-specific input/output operations
#include <fcntl.h>      // required for file control options like open, O_RDWR, O_NONBLOCK, etc.


#define DEVICE_FILE     "/dev/mcp2515"  // mcp2515 device file

std::atomic<bool> stop_thread(false);   // gloabal flag to signal thread to stop  

static void signal_handler(int signal);
static void show_config_menu();
static void show_opmode_menu();
static void config_hw(CanDevice* dev);
static void thread_send_data(CanDevice* dev);
static void thread_receive_data(CanDevice* dev);

int main(int argc, char* argv[]) 
{
    // parse input parameters 
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << "<config|read|write>" << std::endl;
        return 1;
    }

    // register the signal handler for SIGINT and SIGTERM 
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        std::string mode(argv[1]);

        if (mode == "config") {
            CanDevice dev(DEVICE_FILE, O_RDWR);
            config_hw(&dev);
        } 
        else if (mode == "write") {
            // start the sender thread
            CanDevice dev(DEVICE_FILE, O_WRONLY | O_NONBLOCK);
            std::thread sender(thread_send_data, &dev);
            sender.join();  // wait for the sender thread to finish
        } 
        else if (mode == "read") {
            // start the receiver thread
            CanDevice dev(DEVICE_FILE, O_RDONLY | O_NONBLOCK);
            std::thread receiver(thread_receive_data, &dev);
            receiver.join();    // wait for receiver thread finish
        } 
        else {
            std::cerr << "Invalid mode. Use 'read' or 'write'." << std::endl;
        }

    } catch (const std::exception& e) {
    std::cerr << "Exception occurred: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
    }

    return 0;
}


void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
        stop_thread.store(true); // signal threads to stop
        std::cout << "Stop signal recived. Stopping the threads ..." << std::endl;
    }
}

void show_config_menu()
{
    std::cout << "=== MCP2515 Configuration Menu ===\n";
    std::cout << "1. Reset Device\n";
    std::cout << "2. Set RXB1 Filter\n";
    std::cout << "3. Get RXB1 Filter\n";
    std::cout << "4. Set Operation Mode\n";
    std::cout << "5. Get Operation Mode\n";
    std::cout << "6. Exit\n";
    std::cout << "Enter your choice: ";
}

void show_opmode_menu()
{
    std::cout << std::endl;
    std::cout << "=== Setting operation mode ===\n";

    size_t len = get_opmode_strings_len();
    for (size_t i = 0; i < len; i++) {
        std::cout << i <<". " << get_opmode_string(i) << std::endl;
    }
    std::cout << std::endl;
}

void config_hw(CanDevice *dev)
{
    int fd = dev->get_fd();
    std::string input;
    int choice = 0;
    while(true) {
        show_config_menu();
        std::cin >> input;

        try {
            choice = std::stoi(input);
        } catch (const std::exception& e) {
            std::cerr << " Invalid input value, error in: " << e.what() << "\n\n";
            continue;
        }

        switch(choice) {
            case MCP2515_RESET_SEQ_NO: {
                if (ioctl(fd, MCP2515_RESET_IOCTL) < 0) {
                    std::cerr << "Failed to reset the device\n";
                } else {
                    std::cout << "Device reseted successfully and is in configuration mode now\n";
                }
                break;
            }
            case MCP2515_SET_RXBF_SEQ_NO: {   
                rx_filter filter;
                
                std::cout << "\n=== Filter configuration for Receive Buffer 1 ===\n";
                while (true) {
                    std::cout << "\nEnter filter number for RXB1 in range (2 to 5) or 'b' to break: ";
                    std::cin >> input;

                    // Check if the user entered 'b'
                    if (input == "b" || input == "B") {
                        std::cout << "Breaking the filter configuration...\n";
                        break;
                    }

                    // Convert the input string to integer 
                    try {
                        filter.number = static_cast<uint8_t>(std::stoi(input));
                    } catch (...) {
                        std::cout << "Invalid input number. Please try again.\n";
                        continue;
                    }

                    // Check if the entered filter number is in the allowed range 
                    if (filter.number < 2 || filter.number > 5) {
                        std::cout << "Invalid filter number. Please try again.\n";
                        continue;
                    }
                
                    std::cout << "Enter SID filter value in range (0 to 2047) or 'b' to break: ";
                    std::cin >> input;

                    // Check if the user entered 'b'
                    if (input == "b" || input == "B") {
                        std::cout << "Breaking the filter configuration...\n";
                        break;
                    }

                    // Convert the input string to integer 
                    try {
                        filter.value = static_cast<uint8_t>(std::stoi(input));
                    } catch (...) {
                        std::cout << "Invalid input number. Please try again.\n";
                        continue;
                    }

                    if (filter.value < 0 || filter.value > 2047) {
                        std::cout << "Invalid SID filter value. Please try again.\n";
                        continue;
                    }
                
                    if (ioctl(fd, MCP2515_SET_RXBF_IOCTL, &filter)) {
                        std::cerr << "Failed to set RXB1 filter\n";
                    } else {
                        std::cout << "RXB1 filter " << filter.number <<" successfully set\n";
                    }
                }
                
                break;
            }
            case MCP2515_GET_RXBF_SEQ_NO: {
                rx_filter filter;

                std::cout << "\n=== Receive Buffer 1 filter state ===\n";
                for (int i = 2; i <= 5; i++) {
                    filter.number = static_cast<uint8_t>(i);
                    if (ioctl(fd, MCP2515_GET_RXBF_IOCTL, &filter) < 0) {
                        std::cout << "Failed to read the filter value\n";
                        break;
                    } else {
                        std::cout << "Filter " << static_cast<int>(filter.number) << " is set to: " << static_cast<int>(filter.value) << std::endl;
                    }
                }

                break;
            }
            case MCP2515_SET_OPMODE_SEQ_NO: {
                int opmode_input;
                uint8_t opmode = 0;

                // print the operation mode menu 
                show_opmode_menu();

                // read user input 
                std::cout << "Enter operation mode: ";
                std::cin >> opmode_input;   // read the user input as integer first 
                opmode = static_cast<uint8_t>(opmode_input);

                // set the operation mode 
                if (ioctl(fd, MCP2515_SET_OPMODE_IOCTL, &opmode) < 0) {
                    std::cerr << "Failed to set operation mode\n";
                } else {
                    std::cout << "Operation mode set successfully\n";
                }
                break;
            }
            case MCP2515_GET_OPMODE_SEQ_NO: {
                uint8_t opmode = 0;
                if (ioctl(fd, MCP2515_GET_OPMODE_IOCTL, &opmode) < 0) {
                    std::cerr << "Failed to get operation mode\n";
                } else {
                    std::cout << "Current operation mode: " << get_opmode_string(static_cast<uint8_t>(opmode)) << std::endl;
                }
                break;
            }
            case 6: {
                std::cout << "Exiting configuration mode\n";
                return;
            }
            default:   
                std::cout << "Invalid choice, try again\n";
                break;
        }
        std::cout << std::endl;
    }
}

void thread_send_data(CanDevice* dev)
{
    uint16_t sid;
    std::vector<uint8_t> data;
    std::vector<CanFrame> message;
    std::string input;
    fd_set write_fds;
    int fd = dev->get_fd();
    struct timeval timeout;

    while (!stop_thread.load()) {
        // read the CAN-SID 
        std::cout << "CAN-Message to send (type 's' to stop)\n";
        std::cout << "Enter SID in range (0 to 2047): ";
        std::getline(std::cin, input);
        if (input == "s" || input == "S") {
            stop_thread.store(true);
            break;
        }

        try {
            sid =  static_cast<uint16_t>(std::stoi(input)); 
        }
        catch(const std::exception& e) {
            std::cerr << " Invalid SID, error in: " << e.what() << "\n\n";
            continue;
        }
        
        // read the CAN-Data
        std::cout << "Enter message: ";
        std::getline(std::cin, input);
        if (input == "s" || input == "S") {
            stop_thread.store(true);
            break;
        }

        // parse the data input to vector of CAN-Frames
        // std::istringstream iss(input);
        // uint8_t byte = 0x0;
        size_t byte_count = 0;
        data.clear();
        message.clear();
        for (char& byte : input){
            data.push_back(byte);
            byte_count++;

            if ((byte_count % 8) == 0) {
                CanFrame can_frame(sid, data);  // create new frame 
                message.push_back(can_frame);   // message consist of multiple frames
                data.clear();   //  prepare data for the next frame 
            }
        }

        // consider messages that are smaller than 8-byte in the payload 
        if (data.size() !=0) {
            CanFrame can_frame(sid, data);  // create new frame 
            message.push_back(can_frame);   // message consist of multiple frames
            data.clear();   //  prepare data for the next frame 
        }

        // transmit CAN-Message
        for(auto frame : message) {

            // initialize the file descriptor set
            FD_ZERO(&write_fds);
            FD_SET(fd, &write_fds);         // add the device file descriptor 

            // set the timeout value
            timeout.tv_sec = 1; // wait up to 1 second 
            timeout.tv_usec = 0;

             // wait for the device to be ready for writing
            int ret = select(fd + 1, nullptr, &write_fds, nullptr, &timeout);
            if (ret < 0) {
                std::cerr << "select() error" << std::endl;
                break;
            } else if (ret == 0) {
                std::cerr << "Timeout occurred, device is not ready for writing\n";
                continue;
            }

            if (FD_ISSET(fd, &write_fds)) {
                int ret = dev->send_frame(frame);
                frame.print_frame_data();
                if (ret) {
                    std::cerr << "Error: Failed to send message: \n";
                    frame.print_frame_data();
                    break;
                } 
            }
        }
    }
}

void thread_receive_data(CanDevice* dev)
{
    CanFrame frame;
    fd_set read_fds;
    int fd = dev->get_fd();
    struct timeval timeout;

    std::cout << "Waiting for CAN-Message (type 's' to stop)\n";
    while (!stop_thread.load()) {
        // Initialize the file descriptor set
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);          // Add the device file descriptor
        FD_SET(STDIN_FILENO, &read_fds); // Add stdin file descriptor

        // Set timeout to 1 second
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        // Wait for data on either the device or stdin
        int ret = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret < 0) {
            std::cerr << "select() error" << std::endl;
            break;
        } else if (ret == 0) {
            // std::cerr << "Timeout occurred, device is not ready for readable\n";
            continue;
        }

        // Check if data is available on the device
        if (FD_ISSET(fd, &read_fds)) {
            int ret = dev->receive_frame(frame);
            if (ret) {
                std::cerr << "Error: Failed to receive message: \n";
                break;
            } else {
                std::cout << "\nReceived CAN_Message: ";
                frame.print_message();
                frame.print_frame_data();

                std::cout << "Waiting for CAN-Message (type 's' to stop)\n";
            }
        }

        // Check if the user has entered a stop message
        if (FD_ISSET(STDIN_FILENO, &read_fds)) {
            std::string input;
            std::getline(std::cin, input);
            if (input == "s" || input == "S") {
                std::cout << "Stop message received. Stopping thread..." << std::endl;
                stop_thread.store(true);
                break;
            }
        }
    } 
}

