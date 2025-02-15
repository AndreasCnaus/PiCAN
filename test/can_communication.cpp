#include "can_communication.h"
#include <iostream>
#include <string.h>
#include <stdexcept>

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
    can_frame_data fdata = frame.get_frame_data();
    bytes_written = write(m_fd, &fdata, sizeof(fdata));
    if (bytes_written == -1) {
        std::cerr << "Error: Failed to write to the device " << m_file << ": " << strerror(errno) << std::endl;
        return -1;
    }

    // Check if all bytes are written
    if (bytes_written != sizeof(fdata)) {
        std::cerr << "Partial write: " << bytes_written << " of " << sizeof(fdata) << " bytes written" << std::endl;
        return -1;
    }

    return 0;
}

int CanDevice::receive_frame(CanFrame &frame)
{
    ssize_t bytes_read = 0;
    can_frame_data fdata;

    bytes_read = read(m_fd, &fdata, sizeof(fdata));
    if(bytes_read == -1) {
        std::cerr << "Error: Failed to read from the device " << m_file << ": " << strerror(errno) << std::endl;
        return -1;
    }

    frame.set_frame_data(fdata);

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
