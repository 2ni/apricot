#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include "mcu.h"
#include "uart.h"
#include "ssd1306.h"
#include "clock.h"


int main(void) {
  mcu_init();

  SSD1306 ssd1306;
  ssd1306.init();

  char some_string[5] = "Test";

  ssd1306.dot(64, 0);
  ssd1306.largechar('a', 4, 0, 2);
  ssd1306.text(some_string, 0, 0);
  ssd1306.text(some_string, 2, 0);
  ssd1306.text("World", 4, 49); // (128-5*6)/2 = 49
  ssd1306.hline(8, 0, 64, 4); // max width: 128
  clock.sleep_for(8192); // 2sec: 8192 = 2000*32768/8/1000
  ssd1306.off();
  clock.sleep_for(8192);
  ssd1306.on();
  ssd1306.clear(1, 0, 64); // clear horizontal line
  clock.sleep_for(8192);
  ssd1306.largechar('2', 4, 108, 4);
  clock.sleep_for(8192);

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
      ssd1306.hline(8, last_humidity/2, (humidity-last_humidity)/2, 4); // max width: 128
    } else {
      ssd1306.clear(1, humidity/2, (last_humidity-humidity)/2);
    }

    clock.sleep_for(819); // 200msec: 819.2 = 200*32768/8/1000
  }

  DL("done.");
}
