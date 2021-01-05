#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "sleep.h"
#include "pins.h"
#include "mcu.h"

int main(void) {
  mcu_init();

  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output
  while (1) {
    PORTB.OUTTGL = PIN5_bm;
    DL("blink");
    sleep_ms(500);
  }
}
