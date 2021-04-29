#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

int main(void) {
  mcu_init();
  PORTB.DIRSET = PIN1_bm; // set PB1 (led) as output

  while (1) {
    PORTB.OUTTGL = PIN1_bm;
    _delay_ms(500);
    DL("blink");
  }
}
