#include "mcu.h"
#include "pins.h"
#include "uart.h"


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

/*
 * get current voltage of power, aka battery
 * used resistor divider 1M - 220k, 1.5v internale reference
 * returns precision 1/100 volt, eg 495 -> 4.95v
 * max vbat: 8.3v (1.5v ref)
 *           6.1v (1.1v ref)
 *
 * for some strange reasons I get wrong values if prescaler is lower than 128
 */
uint16_t get_vin() {
  pins_vin.port_adc->CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp);
  VREF.CTRLC = VREF_ADC1REFSEL_1V1_gc;

  uint16_t adc = pins_getadc(&pins_vin);
  // (vref * (r1+r2) * precision * adc) / (r2 * adc_precision)
  // (1.5*1220*100*adc) / (220*1024)
  return (134200*adc)/225280; // 1.1v reference
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
