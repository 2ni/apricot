#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include "mcu.h"
#include "uart.h"
#include "ssd1306.h"
#include "sleep.h"


int main(void) {
  mcu_init();
  ssd1306_init();

  char some_string[5] = "Test";

  ssd1306_dot(64, 0);
  ssd1306_char('a', 4, 0);
  ssd1306_text(some_string, 0, 0);
  ssd1306_text("World", 7, 0);
  ssd1306_hline(8, 0, 64, 4); // max width: 128
  sleep_s(2);
  ssd1306_off();
  sleep_s(2);
  ssd1306_on();
  ssd1306_clear(1, 0, 64); // clear horizontal line
  sleep_s(2);

  uint8_t humidity = 0; // value between 0-255
  uint8_t last_humidity = 0;
  int8_t increment = 10;
  while(1) {
    // DF("hum: %u, inc: %i\n", humidity, increment);
    last_humidity = humidity;
    if (((255-humidity) < increment && increment>=0) || (humidity < abs(increment) && increment<0)) {
      increment = -increment;
      humidity = increment > 0 ? 0 : 255;
    } else {
      humidity += increment;
    }

    if (last_humidity < humidity) {
      ssd1306_hline(8, last_humidity/2, (humidity-last_humidity)/2, 4); // max width: 128
    } else {
      ssd1306_clear(1, humidity/2, (humidity-last_humidity)/2);
    }

    sleep_ms(200);
  }

  DL("done.");
}
