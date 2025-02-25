#ifndef MCP2515_CIRCULAR_BUFFER_H
#define MCP2515_CIRCULAR_BUFFER_H

#include "../common/mcp2515_common.h"
#include <linux/mutex.h>

#define BUFFER_SIZE             32   // Size of the Rx Buffer 

struct circular_buffer {
    struct can_frame_data buf[BUFFER_SIZE];
    int head;
    int tail;
    int full;
    struct mutex lock;
};

/* function declarations */
void circular_buffer_init(struct circular_buffer *cb);
int circular_buffer_write(struct circular_buffer *cb, struct can_frame_data *frame);
int circular_buffer_read(struct circular_buffer *cb, struct can_frame_data *frame);
int circular_buffer_is_empty(struct circular_buffer *cb);
int circular_buffer_is_full(struct circular_buffer *cb);
int circular_buffer_reset(struct circular_buffer *cb);

#endif  // MCP2515_CIRCULAR_BUFFER_H