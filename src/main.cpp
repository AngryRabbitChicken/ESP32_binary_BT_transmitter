
#include <Arduino.h>
#include "binary_stream.h"
//#include <global_defs.h>
#include "freertos/ringbuf.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"
#include "BluetoothA2DPSource.h"
#define ONBOARD_LED GPIO_NUM_2
#define SIGIN GPIO_NUM_23
#define TESTPIN GPIO_NUM_3
#define BLTSSID "SSTC Roswita"

RingbufHandle_t buf_handle = NULL;
BluetoothA2DPSource a2dp_source;

static bool input_capture_isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg)
{
  BaseType_t high_task_wakeup = pdFALSE;

  xRingbufferSendFromISR(buf_handle, edata, sizeof(*edata), &high_task_wakeup);

  return high_task_wakeup == pdTRUE;
}

cap_event_data_t *buffer_read(RingbufHandle_t buffer_handle, uint16_t timeout_in_ms)
{
  size_t item_size;
  cap_event_data_t *item = (cap_event_data_t *)xRingbufferReceive(buffer_handle, &item_size, timeout_in_ms);
  if (item != NULL)
  {
    vRingbufferReturnItem(buffer_handle, (void *)item);
  }
  return item;
}
/*
class MCPWM_Stream : public Stream
{
private:
  uint32_t last_edge_time = 0;
  stream_edge_t translate_edge(mcpwm_capture_on_edge_t lce);
  double calc_time_since_last_event(uint32_t current_edge_time);

public:
  using Stream::Stream; // Inherit the constructor
  stream_data_t request_data(uint16_t);
};

double MCPWM_Stream::calc_time_since_last_event(uint32_t current_edge_time)
{
  uint32_t t_diff;

  if (current_edge_time < last_edge_time) // In case there was a timer overflow.
  {
    t_diff = 0xFFFFFFFF - last_edge_time + current_edge_time; // 0xFFFFFFFF is the maximum value of a unit32.
  }
  else
  {
    t_diff = current_edge_time - last_edge_time;
  }
  last_edge_time = current_edge_time;

  return (double)t_diff / rtc_clk_apb_freq_get(); // conversion to seconds
}

stream_edge_t MCPWM_Stream::translate_edge(mcpwm_capture_on_edge_t lce)
{
  if (lce == MCPWM_POS_EDGE)
    return HI;
  else if (lce == MCPWM_NEG_EDGE)
    return LO;
  else
    return SLEEP;
};

stream_data_t MCPWM_Stream::request_data(uint16_t timeout_time_in_ms)
{
  stream_data_t return_val;
  cap_event_data_t *last_capture = buffer_read(buf_handle, timeout_time_in_ms);
  if (last_capture != NULL)
  {
    return_val.edge = HI;//translate_edge(last_capture->cap_edge);
    return_val.t_interval =(double) 0; //calc_time_since_last_event(last_capture->cap_value);
  }
  else
  {
    return_val.edge = SLEEP;
    return_val.t_interval = (double)timeout_time_in_ms * 1e-3;
  }
  return return_val;
};

MCPWM_Stream mcpwm_stream(44100, 10); // first arg: sampling rate in Hz, second arg: timeout time of streaming request in ms
*/
int32_t prep_transmission(Frame *frame, int32_t requested_samples)
{

  for (int32_t current_sample = 0; current_sample < requested_samples; current_sample++)
  {
    uint16_t tx_value = 0; // mcpwm_stream.create_tx_value();

    frame[current_sample].channel1 = tx_value;
    frame[current_sample].channel2 = tx_value;
  }

  return requested_samples;
}

void displ_connection_status()
{
  if (a2dp_source.is_connected())
  {
    digitalWrite(ONBOARD_LED, HIGH);
  }
  else
  {
    digitalWrite(ONBOARD_LED, LOW);
  }
}

void setup()
{
  // Pin setup
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(SIGIN, INPUT_PULLDOWN);

  // Buffer setup
  buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);

  // MCPWM capture setup
  ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, SIGIN)); // Connect MCPWM unit0, MCPWM_CAP_0 input to the GPIO SIGIN
  mcpwm_capture_config_t mcpwm_conf = {
      .cap_edge = MCPWM_BOTH_EDGE,             // Trigger the capture for rising and falling edges
      .cap_prescale = 1,                       // Prescaler for input edges, a prescaler of e.g. 2 will only trigger on each second detected signal edge
      .capture_cb = input_capture_isr_handler, // Function to be executed when a capture event is triggered
      .user_data = NULL                        // Pointer to the user defined ISR function arguments
  };
  ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &mcpwm_conf)); // Enable the capture

  // BT source setup
  a2dp_source.start(BLTSSID, prep_transmission);
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.set_volume(255);
}

void loop()
{
  displ_connection_status();
  delay(200);
}