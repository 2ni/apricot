#include "mcu.h"


void mcu_init() {
  // set prescaler to 2 for 10MHz which is suitable for 3.3v
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);

  // use external crystal as clock (for RTC). Run in standby
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm);
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  // set alternative pins for uart and spi
  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc || PORTMUX_SPI0_ALTERNATE_gc;

  // init uart for debugging
  uart_init();

  // flash led
  pins_flash(&pins_led);
}
