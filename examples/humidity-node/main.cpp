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

#define NUM_PACKETS 5 // we send 2 packets (vcc, debug count, humidity, temperature, new threshold)

#define PER 7
#define INTERVAL_BUTTON         205     // 50ms = 50*32768/(1000*(PER+1))
#define INTERVAL_MEASURE_UPDATE 4096    //  1s  = 1000*32768/(1000*(PER+1))
#define INTERVAL_DISPLAY_OFF    40960   // 10s  = 10*1000*32768/(1000*(PER+1))
#define INTERVAL_LED_STATUS     61440   // 15s  = 15*1000*32768/(1000*(PER+1))
#define INTERVAL_SEND_DATA      3686400 // 15m  = 15*60*1000*32768/(1000*(PER+1))

// packet types we can get from the gateway (upload types) or send to gateway (download types)
namespace TYPE_UPLOAD {
  typedef enum {
    THRESHOLD = 0x0a,   // 1byte (new humidity threshold)
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
HUMIDITYSENSOR sensor(&PA5, &PA6, &PA3); // humidity, pump, temp
TOUCH button(&PA7);
SSD1306 screen;

uint16_t vcc;
uint8_t sensor_humidity = 0;
uint8_t sensor_humidity_prev = 0;
uint8_t sensor_threshold = 50; // 50% on start
uint8_t sensor_threshold_prev = sensor_threshold;
uint8_t sensor_threshold_gw = sensor_threshold;
int16_t sensor_temperature;
uint8_t counter = 0;
uint8_t button_is_long_press = 0;
uint32_t check_button_at = 0;
uint32_t update_screen_at = 0;
uint32_t display_off_at = 0;
uint32_t send_data_at = 1; // ensure we send data at start (0 = off)
uint32_t led_status_at = 1; // ensure we run led status updates (0 = off)
uint8_t responses_len;
RFM69WRAPPER::WPacket packets[NUM_PACKETS];
RFM69WRAPPER::WPacket responses[10];

/*
 * get data from our sensors
 */
void measure(uint16_t *vcc, int16_t *temperature, uint8_t *humidity) {
  pins_enable_buffer(&PA6);

  *vcc = get_vin();
  *temperature = sensor.get_temperature();
  *humidity = (uint8_t)sensor.get_humidity(1);

  pins_disable_buffer(&PA6);
}

/*
 * send data to gw over 868MHz (RFM69HCW)
 */
void send_data() {
  measure(&vcc, &sensor_temperature, &sensor_humidity);
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

  if (sensor_threshold != sensor_threshold_gw) {
    packets[4].type = TYPE_DOWNLOAD::THRESHOLD;
    packets[4].len = 1;
    packets[4].payload[0] = sensor_threshold;
  }

  pins_enable_buffer(&pins_miso);
  rf.send(packets, sensor_threshold != sensor_threshold_gw ? NUM_PACKETS : NUM_PACKETS - 1, responses, &responses_len);
  pins_disable_buffer(&pins_miso);
  for (uint8_t i=0; i<responses_len; i++) {
    switch (responses[i].type) {
      case TYPE_UPLOAD::THRESHOLD:
        sensor_threshold = responses[i].payload[0];
        sensor_threshold_gw = sensor_threshold;
      break;
    }
  }
  counter++;
}

/*
 * show data on lcd screen
 */
void update_screen() {
  if (button_is_long_press) {
    pins_enable_buffer(&PA6);
    sensor_threshold = (uint8_t)sensor.get_humidity(1);
    pins_disable_buffer(&PA6);
    screen.clear(3, 14 + sensor_threshold_prev, 1);
    sensor_threshold_prev = sensor_threshold;
    DF("calibrated: %u%%\n", sensor_threshold);
  }
  measure(&vcc, &sensor_temperature, &sensor_humidity);
  char buf[10] = {0};
  uint8_t len = 0;

  // show humidity
  screen.clear(0, 0, 3*6*2);
  screen.clear(1, 0, 3*6*2);
  len = uart_u2c(buf, sensor_humidity, 0);
  buf[len] = '%';
  buf[len + 1] = '\0';
  screen.text(buf, 0, 0, 2);

  // show temperature
  screen.clear(0, 128-5*6*2, 5*6*2);
  screen.clear(1, 128-5*6*2, 5*6*2);
  len = uart_u2c(buf, sensor_temperature, 1);
  buf[len] = 'C';
  buf[len + 1] = '\0';
  screen.text(buf, 0, 127-(len + 1)*2*6, 2);

  // show vcc
  screen.clear(6, 0, 5*6*2);
  screen.clear(7, 0, 5*6*2);
  len = uart_u2c(buf, vcc, 2);
  buf[len] = 'v';
  buf[len+1] = '\0';
  screen.text(buf, 6, 0, 2);

  // show next update
  if (send_data_at) {
    screen.clear(7, 128-9*6*1, 9*6*1);
    len = uart_sec2human(buf, (send_data_at - clock.current_tick) * (PER + 1) / 32768);
    screen.text(buf, 7, 127 - len*6, 1); // width per char: 6, scale: 1
  }

  // show threshold and humidity bar
  if (sensor_humidity_prev < sensor_humidity) {
    screen.hline(28, 14 + sensor_humidity_prev, sensor_humidity - sensor_humidity_prev, 4);
  } else if (sensor_humidity_prev > sensor_humidity) {
    screen.clear(3, 14 + sensor_humidity, sensor_humidity_prev - sensor_humidity);
  }
  sensor_humidity_prev = sensor_humidity;
  screen.hline(24, 14 + sensor_threshold, 1, 8);
  screen.hline(24, 114, 1, 8);
  screen.hline(24, 14, 1, 8);

  screen.on();
  // DF("screen update: %uv, %i°C, %u%%\n", vcc, sensor_temperature, sensor_humidity);
}

/*
 * display status on onboard led
 * 1 blink: ok
 * 2 blinks: slightly too dry/humid
 * 3 blinks: too dry/humid
 */
void status() {
  pins_enable_buffer(&PA6);
  sensor_humidity = sensor.get_humidity(1);
  pins_disable_buffer(&PA6);
  uint8_t diff = 0;
  uint8_t blinks = 1;
  if (sensor_threshold > sensor_humidity) diff = sensor_threshold - sensor_humidity;
  else if (sensor_humidity > sensor_threshold) diff = sensor_humidity - sensor_threshold;
  if (diff > 20)  blinks = 3;
  else if (diff > 10) blinks = 2;

  DF("status: is %u%% should %u%% (%u)\n", sensor_humidity, sensor_threshold, blinks);
  pins_flash(&pins_led, blinks, 200);
}

/*
 * callback when touch button is released
 */
void cb_button_released(TOUCH::Press_type type, uint32_t ticks) {
  // turn display off in 10 sec starting when button released
  display_off_at = clock.current_tick + INTERVAL_DISPLAY_OFF;
  button_is_long_press = 0;
}

/*
 * callback when touch button reaches long press (and still pressed)
 */
void cb_button_reached(TOUCH::Press_type type) {
  button_is_long_press = 1;
}

/*
 * callback when touch button is pressed (initial press)
 */
void cb_button_pressed() {
  update_screen_at = clock.current_tick + INTERVAL_MEASURE_UPDATE;
  display_off_at = 0; // do not turn off screen as long as button pressed
  led_status_at = 0; // not led status update while screen is on
  update_screen();
}

/*
 * main function (state machine)
 */
int main(void) {
  mcu_init();
  pins_disable_buffer();

  button.init();
  screen.init();
  // only send to  radio if there is one
  pins_enable_buffer(&pins_miso);
  if (rf.init(GATEWAY, NETWORK)) send_data_at = 0;
  pins_disable_buffer(&pins_miso);

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
      led_status_at = clock.current_tick + INTERVAL_LED_STATUS;
      update_screen_at = 0;
      display_off_at = 0;
      screen.off();
      // DL("screen off");
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
