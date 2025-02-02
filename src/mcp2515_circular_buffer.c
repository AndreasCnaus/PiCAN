#include "mcp2515_circular_buffer.h"


void circular_buffer_init(struct circular_buffer *cb)
{
    cb->head = 0;
    cb->tail = 0;
    cb->full = 0;
    mutex_init(&cb->lock);
}

int circular_buffer_write(struct circular_buffer *cb, struct can_message *msg)
{
    int ret = 0;

    mutex_lock(&cb->lock);

    //check if the buffer is full
    if (cb->full) {
        ret = -1;   // buffer full, overflow singla (no error)
        cb->tail = (cb->tail + 1) % BUFFER_SIZE; // move tail to the next position
    }

    cb->buf[cb->head] = *msg;
    cb->head = (cb->head + 1) % BUFFER_SIZE;

    // update the full flag
    cb->full = (cb->head == cb->tail);

    mutex_unlock(&cb->lock);

    return ret;
}

int circular_buffer_read(struct circular_buffer *cb, struct can_message *msg)
{
    if (circular_buffer_is_empty(cb))
        return -1;  // buffer is empty 

    mutex_lock(&cb->lock);

    *msg = cb->buf[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->full = 0;

    mutex_unlock(&cb->lock);

    return 0;
}

int circular_buffer_is_empty(struct circular_buffer *cb)
{
    bool ret = 0;

    mutex_lock(&cb->lock);

    if (cb->head == cb->tail && !cb->full) {
       ret = 1;  // buffer is empty
    } 

    mutex_unlock(&cb->lock);

    return ret;
}

int circular_buffer_is_full(struct circular_buffer *cb)
{
    int ret = 0;

    mutex_lock(&cb->lock);

    ret = cb->full;
    
    mutex_unlock(&cb->lock);

    return ret;
}
