#include <util/delay.h>
#include "pins.h"
#include "touch.h"
#include "uart.h"
#include "millis.h"

TOUCH::TOUCH(pins_t *ipin) {
  pin = ipin;
  set_thresholds(get_avg());
}

TOUCH::TOUCH(pins_t *ipin, uint16_t threshold) {
  pin = ipin;
  set_thresholds(threshold);
}

void TOUCH::set_thresholds(uint16_t ithreshold) {
  threshold = ithreshold;
  threshold_upper = ithreshold + 30;
  threshold_lower = ithreshold + 15;
  // DF("touch threshold: %u\n", threshold);
  // DF("touch threshold upper: %u\n", threshold_upper);
  // DF("touch threshold lower: %u\n", threshold_lower);
}

uint16_t TOUCH::get_avg() {
  uint16_t v, avg = 0;
  for(uint8_t i=0; i<10; i++) {
    v = get_data();
    // DF("v: %u\n", v);
    if (i>4) avg += v;
  }
  avg /= 5;

  return avg;
}

/*
 * get touch value of loaded Ct on Csh
 * returns a value between 0-1023
 * the higher the value the higher Ct
 *
 * not suited for moisture sensor, as Csh will "saturate"
 * and shows almost no difference from 50-100%
 *
 * set input with pullup (charges Ct)
 * connect adc mux to gnd to discharge it
 * start conversion and wait for finishing
 * set input
 * read adc value
 * see https://github.com/martin2250/ADCTouch/blob/master/src/ADCTouch.cpp
 *
 * adc in pin definition must be set
 */
uint16_t TOUCH::get_data() {
  /*
  // to make it a bit faster, don't use function calls:
  // 28us -> 17us
  uint16_t result;
  ADC1.CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_VDDREF_gc | (0<<ADC_SAMPCAP_bp);
  ADC1.CTRLA = ADC_ENABLE_bm | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc;

  ADC1.COMMAND = ADC_STCONV_bm;
  while (!ADC1.INTFLAGS & ADC_RESRDY_bm);
  result = ADC1.RES;

  PORTC.DIRCLR = PIN5_bm;
  PORTC.PIN5CTRL |= PORT_PULLUPEN_bm; // enable pullup
  ADC1.MUXPOS = ADC_MUXPOS_GND_gc;
  _delay_us(5);
  PORTC.PIN5CTRL &= ~PORT_PULLUPEN_bm; // disable pullup

  ADC1.MUXPOS = ADC_MUXPOS_AIN11_gc;
  ADC1.COMMAND = ADC_STCONV_bm;
  while (!ADC1.INTFLAGS & ADC_RESRDY_bm);
  result = ADC1.RES;
  return result;
  */

  uint16_t result;

  (*pin).port_adc->CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_VDDREF_gc | (0<<ADC_SAMPCAP_bp);
  (*pin).port_adc->CTRLA = ADC_ENABLE_bm | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc;

  // for some weird reasons the very 1st adc conversion after sleep is always wrong
  // so we do one dummy conversion here
  (*pin).port_adc->COMMAND = ADC_STCONV_bm;
  while (!((*pin).port_adc->INTFLAGS & ADC_RESRDY_bm));
  result = (*pin).port_adc->RES;

  pins_output(pin, 0);  // set input for high impedance
  pins_pullup(pin, 1);     // set pullup to charge attached Ctouch
  (*pin).port_adc->MUXPOS = ADC_MUXPOS_GND_gc; // discharge Csh
  // _delay_us(10); // not needed for touch sensors
  pins_pullup(pin, 0);    // disable pullup

  // start adc, equalize Ctouch and Csh by setting MUXPOS to AINx and run measurement
  // does not use get_adc to make it faster
  // uint16_t result = get_adc(pin);
  // return result;
  (*pin).port_adc->MUXPOS = ((ADC_MUXPOS_AIN0_gc + (*pin).pin_adc) << 0);
  (*pin).port_adc->COMMAND = ADC_STCONV_bm;
  while (!((*pin).port_adc->INTFLAGS & ADC_RESRDY_bm));
  result = (*pin).port_adc->RES;

  return result;
}

uint8_t TOUCH::is_pressed() {
  uint16_t v = get_data();
  return v > threshold_upper;
}

uint8_t TOUCH::was_pressed() {
  uint16_t v = get_data();

  // finger present
  if (v > threshold_upper && !finger_present) {
    finger_present = 1;
    return 1;
  }

  // finger not present
  if (v < threshold_lower) finger_present = 0;
  return 0;
}

TOUCH::STATUS TOUCH::pressed(uint16_t timeout, pins_t *led) {
  uint32_t now;
  uint16_t v = get_data();

  if (v < threshold_lower && finger_present) {
    finger_present = 0;
  }

  // finger present
  if (v > threshold_upper && !finger_present) {
    now = millis_time();
    if (led) {
      pins_flash(led, 1);
    }
    finger_present = 1;

    // wait for release or timeout
    // TODO put MCU to sleep while waiting
    while (!(get_data() < threshold_lower) && !((millis_time()-now) > timeout));

    return ((millis_time()-now)<timeout ? SHORT : LONG);
  }

  return IDLE;
}
