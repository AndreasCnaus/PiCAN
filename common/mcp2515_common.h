#ifndef MCP2515_COMMON_H
#define MCP2515_COMMON_H

// Include the appropriate header based on the environment
#ifdef __KERNEL__
#include <linux/types.h>    // For uint8_t, uint16_t in kernel space
#else
#include <stdint.h>         // For uint8_t, uint16_t in user space
#endif

#define MCP2515_MAXDL 8     // Maximum Data Length for CAN messages

//  REQOP[2:0]: Request Operation Mode bits
#define MCP2515_NORMAL_MODE         0x00 // 000: Sets Normal Operation mode
#define MCP2515_SLEEP_MODE          0x01 // 001: Sets Sleep mode
#define MCP2515_LOOPBACK_MODE       0x02 // 010: Sets Loopback mode
#define MCP2515_LISTEN_ONLY_MODE    0x03 // 011: Sets Listen-Only mode
#define MCP2515_CONFIG_MODE         0x04 // 100: Sets Configuration mode

struct can_frame_data {
    uint16_t sid;                   // Standard identifier 
    uint8_t dlc;                    // Data length code 
    uint8_t data[MCP2515_MAXDL];    // Data buffer
};

struct rx_filter {
    uint8_t number;
    uint16_t value;
};

#endif // MCP2515_COMMON_H
