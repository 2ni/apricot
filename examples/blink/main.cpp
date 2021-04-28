#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

int main(void) {
  mcu_init();
  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output

  /*
  // in high: ~2.0v
  // in low: 1.2v
  PORTC.DIRCLR = PIN5_bm; // input
  while (1) {
    if (PORTC.IN & PIN5_bm) {
      PORTB.OUTSET = PIN5_bm;
    } else {
      PORTB.OUTCLR = PIN5_bm;
    }
  }
  */

  while (1) {
    PORTB.OUTTGL = PIN5_bm;
    _delay_ms(500);
    DL("blink");
  }
}
