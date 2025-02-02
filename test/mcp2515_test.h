#include <cstdint>  // Includes fixed-width integer types like uint8_t, uint16_t
#include <fcntl.h>  // Includes file control options like open(), close(), read(), write()
#include <unistd.h> // Provides access to the POSIX operating system API
#include <vector>
#include <string>

#define MCP2515_MAXDL           8       // Maximum number of data bytes in Tx or Rx Buffer

struct FrameData {
    uint16_t sid;                       // standard identifier 
    uint8_t dlc;                        // data length code 
    uint8_t data[MCP2515_MAXDL];        // data buffer

    FrameData() = default;                          // default constructor 
    FrameData(const FrameData& other);              // copy constructor   

    FrameData& operator=(const FrameData& other);   // copy assignment operator 
};

class CanFrame {
private:
   FrameData m_fdata;
public: 
    CanFrame() = default;   // default constructor 
    CanFrame(uint16_t sid, std::vector<uint8_t> data);

    FrameData get_frame_data() const { return m_fdata; };
    void set_frame_data(const FrameData& fdata) { m_fdata = fdata; };
    void print_frame_data();
    void print_message();
};

class CanDevice {
private:
    std::string m_file;             // devide file 
    int m_fd;                       // device file descriptor
public:
    CanDevice(std::string file, int oflag);
    ~CanDevice() { close(m_fd); };

    int get_fd() const { return m_fd; };
    int send_frame(const CanFrame &frame);
    int receive_frame(CanFrame &frame);
private:
    int device_open(int oflag);
};