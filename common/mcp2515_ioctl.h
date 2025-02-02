#ifndef MCP2515_IOCTL_H
#define MCP2515_IOCTL_H

#include "mcp2515_common.h"
#include <linux/ioctl.h>

#define MCP2515_MAGIC               'P'
#define MCP2515_RESET_SEQ_NO        0x01
#define MCP2515_SET_RXB1F_SEQ_NO    0x02
#define MCP2515_SET_OPMODE_SEQ_NO   0x03

#define MCP2515_RESET_IOCTL         _IO(MCP2515_MAGIC, MCP2515_RESET_IOCTL)
#define MC2515_SET_RXB1F_IOCTL      _IOW(MCP2515_MAGIC, MCP2515_SET_RXB1F_SEQ_NO, struct rx_filter *filter)
#define MCP2515_SET_OPMODE_IOCTL    _IOW(MCP2515_MAGIC, MCP2515_SET_OPMODE_SEQ_NO, uint8_t mode)

#endif // MCP2515_IOCTL_H