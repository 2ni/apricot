#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "twi.h"


int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // set prescaler to 2 -> 10MHz
  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc || PORTMUX_SPI0_ALTERNATE_gc; // must be set at once, or the mcu might block

  DINIT();
  twi_init();

  uint8_t data[3] = {0};

  DF("read status: %u\n", twi_read_bytes(0x40, data, 0xe3, 3));
  DL("done");
  DF("%u %u\n", data[0], data[1]);

  for (uint8_t addr = 0; addr<128; addr+=1) {
    if (!twi_start(addr)) {
      DF("0x%02x\n", addr);
    }
  }

  DL("done.");
}
