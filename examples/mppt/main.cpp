#include <avr/io.h>
#include "pins.h"
#include "mcu.h"
#include "uart.h"

/*
 * TCD (12bit timer) in two ramp mode
 * TCD only works in idle sleep mode
 * WOA: PA4
 * WOB: PA5
 */

int main(void) {
  mcu_init();

  uint8_t deadtime = 0;   // ms: deadtime/20MHz
  uint8_t duty = 25;      // 0-100
  uint16_t counter = 255; // frq = 20MHz/counter
  TCD0.CTRLA = TCD_CNTPRES_DIV1_gc | TCD_SYNCPRES_DIV1_gc | TCD_CLKSEL_20MHZ_gc;
  TCD0.CTRLB = TCD_WGMODE_TWORAMP_gc;
  _PROTECTED_WRITE(TCD0.FAULTCTRL, TCD_CMPAEN_bm | TCD_CMPBEN_bm);

  TCD0.CMPASET = deadtime;
  TCD0.CMPACLR = counter*duty/100;
  TCD0.CMPBSET = deadtime;
  TCD0.CMPBCLR = counter-TCD0.CMPACLR;

  while (!(TCD0.STATUS & TCD_ENRDY_bm)); // wait for any synch going on
  TCD0.CTRLE = TCD_SYNC_bm;
  TCD0.CTRLA |= TCD_ENABLE_bm;
  while (1);
}
