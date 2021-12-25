#include "pins.h"
#include "uart.h"
#include "mcu.h"
#include "rfm69.h"
#include "rfm69wrapper.h"
#include "humidity_sensor.h"
#include "touch.h"
#include "ssd1306.h"

#define GATEWAY 99
#define NETWORK 33

#define NUM_PACKETS 4 // we send 2 packets (vcc, debug count, humidity, temperature)

#define PER 7
#define INTERVAL_BUTTON         32768/(1000*(PER+1))*50         // 50ms
#define INTERVAL_MEASURE_UPDATE 32768/(1000*(PER+1))*1000       // 1s
#define INTERVAL_DISPLAY_OFF    32768/(1000*(PER+1))*1000*10    // 10s
#define INTERVAL_LED_STATUS     32768/(1000*(PER+1))*1000*15    // 15s
#define INTERVAL_SEND_DATA      32768/(1000*(PER+1))*1000*60*15 // 15min
//*

// packet types we can get from the gateway (upload types) or send to gateway (download types)
namespace TYPE_UPLOAD {
  typedef enum {
    TIMESTAMP = 0x01,
  } Type_upload;
}

namespace TYPE_DOWNLOAD {
  typedef enum {
    DBG = 0x00,         // 1byte
    VCC = 0x01,         // 2bytes
    TEMP = 0x02,        // 2bytes
    RSSI = 0x03,        // reserved (2bytes)
    SENSOR_HUM = 0x08,  // 1byte (relative humidity)
    SENSOR_TEMP = 0x09, // 2bytes
    THRESHOLD = 0x0a,   // 1byte (new humidity threshold)
  } Type_download;
}

RFM69WRAPPER rf;
HUMIDITYSENSOR sensor(&PA5, &PA6, &PA3);
TOUCH button(&PA7);
SSD1306 screen;

uint16_t vcc;
uint8_t sensor_humidity;
uint8_t sensor_threshold = 50; // 50% on start
int16_t sensor_temperature;
uint8_t counter = 0;
uint8_t button_is_long_press = 0;
uint32_t timestamp;
uint32_t check_button_at = 0;
uint32_t update_screen_at = 0;
uint32_t display_off_at = 0;
uint32_t send_data_at = 1; // ensure we send data at start (0 = off)
uint32_t led_status_at = 1; // ensure we run led status updates (0 = off)
uint8_t responses_len;
RFM69WRAPPER::WPacket packets[NUM_PACKETS];
RFM69WRAPPER::WPacket responses[10];

void measure(uint16_t *vcc, int16_t *temperature, uint8_t *humidity) {
  *vcc = get_vin();
  *temperature = sensor.get_temperature();
  *humidity = (uint8_t)sensor.get_humidity(1);
}

void send_data() {
  measure(&vcc, &sensor_temperature, &sensor_humidity);
  DF("send data: %uv, %i°C, %u%%\n", vcc, sensor_temperature, sensor_humidity);
  return;
  packets[0].type = TYPE_DOWNLOAD::VCC;
  packets[0].len = 2;
  packets[0].payload[0] = vcc >> 8;
  packets[0].payload[1] = vcc & 0xff;
  packets[1].type = TYPE_DOWNLOAD::DBG;
  packets[1].len = 1;
  packets[1].payload[0] = counter;
  packets[2].type = TYPE_DOWNLOAD::SENSOR_HUM;
  packets[2].len = 1;
  packets[2].payload[0] = sensor_humidity;
  packets[3].type = TYPE_DOWNLOAD::SENSOR_TEMP;
  packets[3].len = 2;
  packets[3].payload[0] = sensor_temperature >> 8;
  packets[3].payload[1] = sensor_temperature & 0xff;

  rf.send(packets, NUM_PACKETS, responses, &responses_len);
  for (uint8_t i=0; i<responses_len; i++) {
    switch (responses[i].type) {
      case TYPE_UPLOAD::TIMESTAMP:
        timestamp = ((uint32_t)responses[i].payload[0] << 24)
              | ((uint32_t)responses[i].payload[1] << 16)
              | ((uint16_t)responses[i].payload[2] << 8)
              | responses[i].payload[3];

        DF("  " OK("timestamp: %lu") "\n", timestamp);
      break;
      default:
        DF(NOK("unknown type: 0x%02x") "\n", responses[i].type);
        uart_arr("", responses[i].payload, responses[i].len);
      break;
    }
  }
  counter++;
}

void update_screen() {
  if (button_is_long_press) {
    sensor_threshold = (uint8_t)sensor.get_humidity(1);
    DF("calibrated: %u%%\n", sensor_threshold);
  }
  measure(&vcc, &sensor_temperature, &sensor_humidity);
  screen.on();
  DF("screen update: %uv, %i°C, %u%%\n", vcc, sensor_temperature, sensor_humidity);
}

void status() {
  sensor_humidity = sensor.get_humidity(1);
  uint8_t diff = 0;
  uint8_t blinks = 1;
  if (sensor_threshold > sensor_humidity) diff = sensor_threshold - sensor_humidity;
  else if (sensor_humidity > sensor_threshold) diff = sensor_humidity - sensor_threshold;
  if (diff > 20)  blinks = 3;
  else if (diff > 10) blinks = 2;

  DF("status: is %u%% should %u%% (%u)\n", sensor_humidity, sensor_threshold, blinks);
  pins_flash(&pins_led, blinks, 200);
}

void cb_button_released(TOUCH::Press_type type, uint32_t ticks) {
  // turn display off in 10 sec starting when button released
  display_off_at = clock.current_tick + INTERVAL_DISPLAY_OFF;
  button_is_long_press = 0;
}

void cb_button_reached(TOUCH::Press_type type) {
  DL("long press reached");
  button_is_long_press = 1;
}

void cb_button_pressed() {
  update_screen_at = clock.current_tick + INTERVAL_MEASURE_UPDATE;
  display_off_at = 0; // do not turn off screen as long as button pressed
  led_status_at = 0; // not led status update while screen is on
  update_screen();
}

int main(void) {
  mcu_init();
  // pins_disable_buffer();

  button.init();
  screen.init();
  rf.init(GATEWAY, NETWORK);

  measure(&vcc, &sensor_temperature, &sensor_humidity); // initial humidity measure is always 0%

  while (1) {
    check_button_at = clock.current_tick + INTERVAL_BUTTON;
    button.is_pressed(&cb_button_released, &cb_button_pressed, &cb_button_reached, &pins_led, 12288, 0);

    // measure + update screen
    if (update_screen_at && clock.current_tick >= update_screen_at) {
      update_screen_at = clock.current_tick + INTERVAL_MEASURE_UPDATE;
      update_screen();
    }

    // display off
    if (display_off_at && clock.current_tick >= display_off_at) {
      update_screen_at = 0;
      display_off_at = 0;
      led_status_at = clock.current_tick + INTERVAL_LED_STATUS;
      screen.off();
      DL("screen off");
    }

    // led status
    if (led_status_at && clock.current_tick >= led_status_at) {
      led_status_at = clock.current_tick + INTERVAL_LED_STATUS;
      status();
    }

    // send to gw (only if no touch activity)
    if (!update_screen_at && send_data_at && clock.current_tick >= send_data_at) {
      send_data_at = clock.current_tick + INTERVAL_SEND_DATA;
      send_data();
    }

    clock.sleep_until(check_button_at);
  }
}
