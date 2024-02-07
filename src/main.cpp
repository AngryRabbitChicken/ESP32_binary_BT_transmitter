#include <Arduino.h>
#define DEBUG 1 // DEBUG 0 -> release; DEBUG 1 -> Transmitter testing; DEBUG 2 -> Core functionalities w/o transmitter
// include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"

#if DEBUG <= 1
#include "BluetoothA2DPSource.h"
#endif

#define ONBOARD_LED GPIO_NUM_2
#define SIGIN GPIO_NUM_23
#define TESTPIN GPIO_NUM_3
#define BLTSSID "SSTC Roswita"

enum
{
  IDLE,
  WAKEUP,
  ACTIVE
} TX_STATUS;

RingbufHandle_t buf_handle = NULL;

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

#if DEBUG >= 2
UBaseType_t buffer_count_items(RingbufHandle_t buffer_handle)
{
  UBaseType_t items_in_buffer;
  vRingbufferGetInfo(buffer_handle, NULL, NULL, NULL, NULL, &items_in_buffer);
  return items_in_buffer;
}
#endif

#if DEBUG <= 1
BluetoothA2DPSource a2dp_source;

double calc_time_since_last_event(uint32_t current_edge_time)
{
  static uint32_t last_edge_time = 0;
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

  return (double) t_diff / rtc_clk_apb_freq_get(); // conversion to seconds
}

int16_t tx_value_from_edge(mcpwm_capture_on_edge_t lce)
{
  int16_t tx_value = -32768;
  if (lce == MCPWM_NEG_EDGE) // If our last Last Caught Edge (lce) was negative, we were positive all the time until the edge.
  {
    tx_value = 32767;
  }
  return tx_value;
}

int16_t create_tx_value(uint32_t sampling_rate_in_hz, uint16_t timeout_time_in_ms)
{
  static double current_time = 0;
  static double time_of_last_event = 0;
  static mcpwm_capture_on_edge_t lce = MCPWM_POS_EDGE;
  double t_delta = (double) 1 / sampling_rate_in_hz;
  

  if(current_time > time_of_last_event)
  {
    cap_event_data_t *last_capture = buffer_read(buf_handle, timeout_time_in_ms);
    if(last_capture != NULL)
    {
      lce = last_capture->cap_edge;
      time_of_last_event += calc_time_since_last_event(last_capture->cap_value);
      if (TX_STATUS == IDLE)
      {
        TX_STATUS = WAKEUP;
      }
    }
    else
    {
      time_of_last_event += (double) timeout_time_in_ms * 1e-3;
      TX_STATUS = IDLE;
    }
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
    if (lce == MCPWM_NEG_EDGE) // If our last Last Caught Edge (lce) was negative, we were positive all the time until the edge.
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

int32_t prep_transmission(Frame *frame, int32_t requested_samples)
{
  uint32_t sampling_rate_in_hz = 44100;
  uint16_t timeout_time_in_ms = 10;

  for(int32_t current_sample = 0; current_sample < requested_samples; current_sample++)
  {
      uint16_t tx_value = create_tx_value(sampling_rate_in_hz, timeout_time_in_ms);

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
#endif

void setup()
{
#if DEBUG > 0
  Serial.begin(115200);
  Serial.println("Starting Binary BLT transmitter in debug mode.");
  Serial.print("The current clock frequency in Hz is: ");
  Serial.println(rtc_clk_apb_freq_get());
#endif

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
#if DEBUG <= 1
  a2dp_source.start(BLTSSID, prep_transmission);
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.set_volume(255);
  TX_STATUS = ACTIVE;
#endif
}

void loop()
{

#if DEBUG <= 1
  displ_connection_status();
  delay(200);
#endif

#if DEBUG >= 2
  cap_event_data_t *cap_event_dummy;
  while (buffer_count_items(buf_handle) < 1)
  {
    NOP();
  }
  while (buffer_count_items(buf_handle) > 0)
  {
    cap_event_dummy = buffer_read(buf_handle, 100);
  }
  Serial.print("Current edge type: ");
  Serial.println(cap_event_dummy->cap_edge);
  Serial.print("Time since last edge in us: ");
  Serial.println((double)cap_event_dummy->cap_value * 1e6 / rtc_clk_apb_freq_get());
  delay(1000);
#endif
}