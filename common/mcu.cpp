#include "mcu.h"
#include "pins.h"
#include "uart.h"
#include "clock.h"

CLOCK clock;

void mcu_init(uint8_t enable_rx, uint8_t enable_clock) {
  // set prescaler to 2 for 10MHz which is suitable for 3.3v
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (F_CPU == 10000000 ? CLKCTRL_PDIV_2X_gc : (F_CPU == 5000000 ? CLKCTRL_PDIV_4X_gc : CLKCTRL_PDIV_8X_gc)) | (F_CPU == 20000000 ? 0 : CLKCTRL_PEN_bm));

  // use external crystal as clock (for RTC). Run in standby
#ifdef __AVR_ATtiny3217__
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm);
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  // set alternative pins for uart and spi
  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc;

#elif defined(__AVR_ATtiny1604__)
  // set alternative pins for uart
  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;
#endif

  // sleep command puts mcu in standby (idle, standby, power down)
  SLPCTRL.CTRLA = (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm);

  // start rtc
  if (enable_clock) {
    clock.init();
  }

  // init uart for debugging
  uart_init(enable_rx);

  // flash led
  pins_flash(&pins_led);
}

/*
 * get current voltage of power, aka battery
 * used resistor divider 1M - 220k, 1.5v internal reference
 * returns precision 1/100 volt, eg 495 -> 4.95v
 * max vbat: 8.3v (1.5v ref)
 *           6.1v (1.1v ref)
 */
uint16_t get_vin() {
  uint16_t adc = 0;
  for (uint8_t i=0; i<4; i++) {
    adc += (134200*pins_getadc(&pins_vin, VREF_ADC0REFSEL_1V1_gc))/225280;
  }
  return adc/4;

  // (vref * (r1+r2) * precision * adc) / (r2 * adc_precision)
  // (1.5*1220*100*adc) / (220*1024)
  // return (134200*adc)/225280; // 1.1v reference
  // return (183000*adc)/225280; // 1.5v reference
}

/*
 * measures vcc of chip
 * based on http://ww1.microchip.com/downloads/en/Appnotes/00002447A.pdf
 * returns 1/100 volt precision as int
 * eg 316 -> 3.16v
 */
uint16_t get_vcc_mcu() {
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;   // set pin to int ref

  ADC0.CTRLC = ADC_PRESC_DIV128_gc       // 10MHz with prescaler 128 -> 78kHz
    | ADC_REFSEL_VDDREF_gc              // VDD
    | (0<<ADC_SAMPCAP_bp);              // disable sample capacitance

  ADC0.CTRLA = (1<<ADC_ENABLE_bp)       // enable adc
    | (0<<ADC_FREERUN_bp)               // no free run
    | ADC_RESSEL_10BIT_gc;              // set resolution to 10bit

  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc; // 1.1v reference

  ADC0.COMMAND |= 1;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  ADC0.CTRLA = 0;                       // disable adc
  return (112640 / ADC0.RES);           // 1024*1.1*100
}
