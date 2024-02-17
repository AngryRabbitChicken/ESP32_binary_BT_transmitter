

#include <stdint.h>
#include "binary_stream.h"

Stream::Stream(uint32_t sampling_rate_in_hz, uint16_t timeout_time_in_ms) : timeout_time_in_ms(timeout_time_in_ms)
{
    t_delta = (double)1 / sampling_rate_in_hz;
}

void Stream::prep_transmission()
{
    stream_data_t last_capture = request_data(timeout_time_in_ms);
    if (last_capture.edge != SLEEP)
    {
        last_caught_edge = last_capture.edge;
        time_of_last_event += last_capture.t_interval;
        if (TX_STATUS == IDLE)
        {
            TX_STATUS = WAKEUP;
        }
    }
    else
    {
        last_caught_edge = LO;
        time_of_last_event += (double)timeout_time_in_ms * 1e-3;
        TX_STATUS = IDLE;
    }
};

int16_t Stream::create_tx_value()
{
    if (current_time > time_of_last_event)
    {
        prep_transmission();
    }

    int16_t tx_value = -32768;
    switch (TX_STATUS)
    {
    case IDLE:
        current_time += t_delta;
        break;

    case WAKEUP:
        current_time = 0;
        time_of_last_event = 0;
        TX_STATUS = ACTIVE;

    case ACTIVE:
        if (last_caught_edge == LO) // If our last Last Caught Edge (lce) was negative, we were positive all the time until the edge.
        {
            tx_value = 32767;
        }
        current_time += t_delta;
        break;

    default:
        tx_value = 0; // In case wierd things happen, we send an intermediate value
    }
    return tx_value;
}