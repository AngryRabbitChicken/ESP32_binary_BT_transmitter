#include <binary_stream.h>

GenericBTStream::GenericBTStream(uint32_t sampling_rate_in_hz, uint16_t timeout_time_in_ms) : timeout_time_in_ms(timeout_time_in_ms)
{
    t_delta = (double) 1 / sampling_rate_in_hz;
}

int16_t GenericBTStream::create_tx_value()
{
    if (current_time > time_of_last_event)
    {
        request_data(timeout_time_in_ms);
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