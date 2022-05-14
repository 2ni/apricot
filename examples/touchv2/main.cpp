#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

/*
 * get_data_self()
 * self capacitance with internal s/h capacitance
 *   connect touch to PA7 (= AIN7)
 *   connect active shield to PA4
 *   one cycle transfer loading then measure voltage on s/h
 *   if value higher -> finger is there
 *   (the touch sensor seems to have ~1/2 capacity of internal s/h)
 *
 *
 * get_data_charge_transfer()
 *
 *               1k
 *          +---/\/\/---+---- PA5
 *          |           |
 *          |           |
 *  Ctouch ===         === Csense ~22nF
 *          •           |
 *          •           |
 *         •••          +---- PA6
 *         GND
 *
 *  1) discharge Csense, Ctouch: PA5=PA6=0 (output)
 *  2) disconnect Csense       : PA6=tristate (=input)
 *  3) charge ctouch           : PA5=1 (output)
 *  4) disconnect charge       : PA5=tristate (=input)
 *  5) transfer load to Csense : PA6=0 (output)
 *  6) increment count / check if PA5 high
 *
 *  if count lower -> finger is there (Ctouch gets bigger and needs less cycles to load Csense)
 *
 *  based on https://www.avrfreaks.net/forum/charge-transfer-capacitance-measurement
 */

uint16_t get_data_self() {
  ADC0.CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_VDDREF_gc | (0<<ADC_SAMPCAP_bp);
  ADC0.CTRLA = ADC_ENABLE_bm | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc;

  // active shield
  PORTA.DIRSET = PIN4_bm;

  // disable input buffer for better adc (according to p.429)
  PORTA.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTA.OUTSET = PIN7_bm;

  ADC0.MUXPOS = ADC_MUXPOS_GND_gc; // empty s/h capacitor

  // charge Ctouch by setting output. set active shield?
  PORTA.OUTSET = PIN4_bm; // activate shield
  PORTA.DIRSET = PIN7_bm;
  __builtin_avr_delay_cycles(1);
  PORTA.DIRCLR = PIN7_bm; // high impedance. clr active shield?
  PORTA.OUTCLR = PIN4_bm; // deactivate shield

  // transfer charge to Csh
  ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;

  // read s/h voltage
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!ADC0.INTFLAGS & ADC_RESRDY_bm);

  return ADC0.RES;
}

uint16_t get_data_charge_transfer() {
  uint16_t count = 0;

  // discharge Csense, Chum
  PORTA.OUTCLR = PIN5_bm;
  PORTA.OUTCLR = PIN6_bm;
  PORTA.DIRSET = PIN5_bm;
  PORTA.DIRSET = PIN6_bm;
  __builtin_avr_delay_cycles(5); // probably not needed to wait much here

  while ((PORTA.IN & PIN5_bm) == 0) {
    // disconnect Csense by setting Csense_gnd to tristate (=input)
    PORTA.DIRCLR = PIN6_bm;

    // charge Chum
    PORTA.OUTSET = PIN5_bm; // set active shield?
    PORTA.DIRSET = PIN5_bm;
    __builtin_avr_delay_cycles(5);

    // disconnect charge by setting to tristate (=input) and clear active shield?
    PORTA.DIRCLR = PIN5_bm;

    // transfer load from Chum to Csense and check if threshold reached
    PORTA.DIRSET = PIN6_bm;
    __builtin_avr_delay_cycles(5);
    count++;
  }
  PORTA.DIRSET = PIN5_bm;
  PORTA.OUTCLR = PIN5_bm;

  return count;
}

int main(void) {
  mcu_init();

  while (1) {
    // DF("res: %u\n", get_data_charge_transfer());
    DF("res: %u\n", get_data_self());
    _delay_ms(1000);
  }
  //*/
}
