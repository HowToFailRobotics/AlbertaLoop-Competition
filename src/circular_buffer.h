#pragma once


#define NUM_BITS    4
#define BUFFER_LEN  (1 << NUM_BITS)

struct circular_buffer_t {
    int values[BUFFER_LEN];
    unsigned int write_ptr = 0;
    unsigned int read_ptr = 0;
    int count = 0;
} ;
typedef circular_buffer_t circular_buffer;

void push(int val, volatile circular_buffer *buf);

int pop(volatile circular_buffer *buf);