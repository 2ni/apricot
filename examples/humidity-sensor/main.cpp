#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"
#include "humidity_sensor.H"

HUMIDITYSENSOR sensor(&PA5, &PA6);

int main(void) {
  mcu_init();

  while (1) {
    uint16_t h = sensor.get_humidity();
    DF("humidity: %u Vin: %u\n", h, get_vin());
    _delay_ms(1000);
  }
}
