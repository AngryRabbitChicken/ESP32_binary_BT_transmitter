#ifndef BINARY_STREAM_H
#define BINARY_STREAM_H

#include <stdint.h>

typedef enum
{
    HI,
    LO
} stream_edge_t;

class GenericBTStream
{
private:
    double current_time = 0;    
    double t_delta;
    uint16_t timeout_time_in_ms;    
    
    //void prep_transmission();

public:
    GenericBTStream(uint32_t sampling_rate_in_hz, uint16_t timeout_time_in_ms);

    double time_of_last_event = 0;
    stream_edge_t last_caught_edge = HI;
    enum
    {
        IDLE,
        WAKEUP,
        ACTIVE
    } TX_STATUS;

    int16_t create_tx_value();
    virtual void request_data(uint16_t timeout_time_in_ms) = 0;
};

#endif