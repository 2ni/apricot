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


  char out[6];
  uart_u2c(out, sht20.get_temperature(), 2);
  DF("temp: %sC\n", out);
  uart_u2c(out, sht20.get_humidity(), 2);
  DF("hum: %s%%\n", out);

  uart_u2c(out, sht20.get_temperature(1), 2);
  DF("temp: %sC\n", out);

  uart_u2c(out, sht20.get_humidity(1), 2);
  DF("hum: %s%%\n", out);
}
