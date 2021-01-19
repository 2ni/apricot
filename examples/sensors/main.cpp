#include <util/delay.h>
#include <avr/io.h>
#include <util/delay.h>

#include "uart.h"
#include "mcu.h"
#include "sht20.h"
#include "twi.h"
#include "millis.h"

SHT20 sht20;

int main(void) {
  mcu_init();

  sht20.init();
  millis_init();

  char out[6];
  uint32_t now = millis_time();

  uart_u2c(out, sht20.get_temperature(), 2);
  DF("temp nohold: %sC (%lums)\n", out, millis_time()-now);

  now = millis_time();
  uart_u2c(out, sht20.get_humidity(), 2);
  DF("hum: nohold %s%% (%lums)\n", out, millis_time()-now);

  sht20.set_resolution(USER_REG_PRECISION_RH11_TEMP11);

  now = millis_time();
  uart_u2c(out, sht20.get_temperature(), 2);
  DF("temp nohold 11bit: %sC (%lums)\n", out, millis_time()-now);

  now = millis_time();
  uart_u2c(out, sht20.get_humidity(), 2);
  DF("hum nohold 11bit: %s%% (%lums)\n", out, millis_time()-now);

  sht20.set_resolution(USER_REG_PRECISION_RH12_TEMP14);

  now = millis_time();
  uart_u2c(out, sht20.get_temperature(1), 2);
  DF("temp hold: %sC (%lums)\n", out, millis_time()-now);

  now = millis_time();
  uart_u2c(out, sht20.get_humidity(1), 2);
  DF("hum hold: %s%% (%lums)\n", out, millis_time()-now);
}
