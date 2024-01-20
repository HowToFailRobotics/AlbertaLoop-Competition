#include "circular_buffer.h"

void push(int val, volatile circular_buffer *buf)
{
  if (buf->count < BUFFER_LEN)
  {
    buf->values[buf->write_ptr] = val;
    buf->write_ptr++;
    buf->write_ptr &= BUFFER_LEN-1;
    buf->count++;
  }
}

int pop(volatile circular_buffer *buf)
{
  if (buf->read_ptr != buf->write_ptr)
  {
    int ret = buf->values[buf->read_ptr];
    buf->read_ptr++;
    buf->read_ptr &= BUFFER_LEN-1;
    buf->count--;
    return ret;
  }
  else
  {
    return -1;
  }
}