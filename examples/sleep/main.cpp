#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "sleep.h"

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // set prescaler to 2 -> 10MHz
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm); // external crystal, runstdby, enable
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc || PORTMUX_SPI0_ALTERNATE_gc; // must be set at once, or the mcu might block

  uart_init();

  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output
  while (1) {
    PORTB.OUTTGL = PIN5_bm;
    DL("blink");
    sleep_ms(500);
  }
}
