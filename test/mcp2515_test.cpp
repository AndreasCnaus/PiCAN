#include "mcp2515_test.h"
#include <iostream>     // required the standard C++ input/output stream library
#include <stdexcept>    // required for exceptions 
#include <cstring>      // required for strerror
#include <cerrno>       // required errno
#include <atomic>       // required for atomic variable to stop threads 
#include <mutex>        // required for std::mutex and std::lock_guard
#include <sstream>
#include <thread>
#include <csignal>

#define DEVICE_FILE     "/dev/mcp2515"  // mcp2515 device file

std::atomic<bool> stop_thread(false);   // gloabal flag to signal thread to stop  

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
        stop_thread.store(true); // signal threads to stop
        std::cout << "Stop signal recived. Stopping the threads ..." << std::endl;
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
        std::cout << "Enter SID in hexadecimal (0x###): ";
        std::getline(std::cin, input);
        if (input == "s") {
            stop_thread.store(true);
            break;
        }
        sid = static_cast<uint16_t>(std::stoul(input, nullptr, 16));   // interprets an unsigned integer value in the input string

        // read the CAN-Data
        std::cout << "Enter message: ";
        std::getline(std::cin, input);
        if (input == "s") {
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
                std::cout << "Received CAN_Message: ";
                frame.print_message();
                frame.print_frame_data();
            }
        }

        // Check if the user has entered a stop message
        if (FD_ISSET(STDIN_FILENO, &read_fds)) {
            std::string input;
            std::getline(std::cin, input);
            if (input == "s") {
                std::cout << "Stop message received. Stopping thread..." << std::endl;
                stop_thread.store(true);
                break;
            }
        }
    } 
}

int main(int argc, char* argv[]) 
{
    // parse input parameters 
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << "<read|write>" << std::endl;
        return 1;
    }

    // register the signal handler for SIGINT and SIGTERM 
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        
        std::string mode(argv[1]);

        if (mode == "write") {
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

FrameData::FrameData(const FrameData &other)
{
    sid = other.sid;
    dlc = other.dlc;
    std::memcpy(data, other.data, sizeof(data));

}

FrameData &FrameData::operator=(const FrameData &other)
{
    if (this != &other) {
        sid = other.sid;
        dlc = other.dlc;
        std::memcpy(data, other.data, sizeof(data));
    }
    return *this;
}

CanFrame::CanFrame(uint16_t sid, std::vector<uint8_t> data)
{
    // Check if the data size exceeds the maximum allowed length
    if (data.size() > MCP2515_MAXDL) {
        throw std::invalid_argument("Data size exceeds the maximum allowed length of CAN frame payload");
    }

    m_fdata.sid = sid;
    m_fdata.dlc = data.size();
    std::copy(data.begin(), data.end(), m_fdata.data);
}

void CanFrame::print_frame_data()
{
    std::cout << "SID: " << m_fdata.sid << std::endl;
    std::cout << "DLC: " << static_cast<int>(m_fdata.dlc) << std::endl;
    std::cout << "Data: ";
    for (size_t i = 0; i < m_fdata.dlc; ++i) {
        std::cout << static_cast<int>(m_fdata.data[i]) << " ";
    }
    std::cout << "\n\n";
}

void CanFrame::print_message()
{
    for (size_t i = 0; i < m_fdata.dlc; ++i) {
        std::cout << static_cast<char>(m_fdata.data[i]);
    }
    std::cout << std::endl;
}

CanDevice::CanDevice(std::string file, int oflag)
: m_file(file)
{
    if (device_open(oflag) != 0) {
        throw std::runtime_error("Failed to open device" + m_file);
    }
}

int CanDevice::send_frame(const CanFrame &frame)
{
    ssize_t bytes_written = 0;

    // write data to the device
    FrameData frame_data = frame.get_frame_data();
    bytes_written = write(m_fd, &frame_data, sizeof(frame_data));
    if (bytes_written == -1) {
        std::cerr << "Error: Failed to write to the device " << m_file << ": " << strerror(errno) << std::endl;
        return -1;
    }

    // Check if all bytes are written
    if (bytes_written != sizeof(frame_data)) {
        std::cerr << "Partial write: " << bytes_written << " of " << sizeof(frame_data) << " bytes written" << std::endl;
        return -1;
    }

    return 0;
}

int CanDevice::receive_frame(CanFrame &frame)
{
    ssize_t bytes_read = 0;
    FrameData frame_data;

    bytes_read = read(m_fd, &frame_data, sizeof(frame_data));
    if(bytes_read == -1) {
        std::cerr << "Error: Failed to read from the device " << m_file << ": " << strerror(errno) << std::endl;
        return -1;
    }

    frame.set_frame_data(frame_data);

    return 0;
}

int CanDevice::device_open(int oflag)
{
    m_fd = open(m_file.c_str(), oflag);
    if (m_fd == -1) {
        return -1;
    }

    return 0;
}

