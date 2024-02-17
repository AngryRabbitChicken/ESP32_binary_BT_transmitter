#ifndef BINARY_STREAM_H
#define BINARY_STREAM_H

#include <stdint.h>
struct cap_event_data_t;

typedef enum
{
    HI,
    LO,
    SLEEP
} stream_edge_t;

typedef struct
{
    stream_edge_t edge;
    double t_interval;
} stream_data_t;

class Stream
{
private:
    double current_time = 0;
    double time_of_last_event = 0;
    double t_delta;
    uint16_t timeout_time_in_ms;
    stream_edge_t last_caught_edge = HI;
    enum
    {
        IDLE,
        WAKEUP,
        ACTIVE
    } TX_STATUS;
    void prep_transmission();

public:
    Stream(uint32_t sampling_rate_in_hz, uint16_t timeout_time_in_ms);
    int16_t create_tx_value();
    virtual stream_data_t request_data(uint16_t timeout_time_in_ms) = 0;
};

#endif