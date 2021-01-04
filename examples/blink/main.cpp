#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // set prescaler to 2 -> 10MHz
  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc || PORTMUX_SPI0_ALTERNATE_gc; // must be set at once, or the mcu might block

  DINIT();

  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output
  while (1) {
    PORTB.OUTTGL = PIN5_bm;
    _delay_ms(500);
    DL("blink");
  }
}
