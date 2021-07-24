#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"
#include "humidity_sensor.H"

HUMIDITYSENSOR sensor(&PA5, &PA6, &PA3);

int main(void) {
  mcu_init();

  while (1) {
    uint16_t t = sensor.get_temperature();
    uint16_t h = sensor.get_humidity();
    DF("humidity: %u%% (%u) temp: %u vin: %u\n",
      sensor.to_relative(h, sensor.characteristics_humidity, sensor.characteristics_humidity_len),
      h,
      t,
      get_vin()
    );

    _delay_ms(1000);
  }
}
