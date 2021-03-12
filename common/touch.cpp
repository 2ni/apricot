#include <util/delay.h>
#include "pins.h"
#include "touch.h"
#include "uart.h"
#include "mcu.h"

TOUCH::TOUCH(pins_t *ipin, uint16_t low, uint16_t high, uint16_t idle) {
  pin = ipin;
  if (idle == 0) {
    idle = get_avg();
  }
  set_thresholds(idle+low, idle+high);
}

void TOUCH::set_thresholds(uint16_t low, uint16_t high) {
  threshold_lower = low;
  threshold_upper = high;
  DF("touch threshold lower: %u\n", threshold_lower);
  DF("touch threshold upper: %u\n", threshold_upper);
}

uint16_t TOUCH::get_avg() {
  uint16_t v, avg = 0;
  for(uint8_t i=0; i<16; i++) {
    v = get_data();
    // DF("v: %u\n", v);
    if (i>7) avg += v;
  }
  avg /= 8;

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

uint8_t TOUCH::is_pressed(void (*fn)(Press_type, uint32_t), uint32_t tick_long, uint32_t tick_verylong) {
  return is_pressed(fn, &pins_led, tick_long, tick_verylong);
}

uint8_t TOUCH::is_pressed(void (*fn)(Press_type, uint32_t), pins_t *led, uint32_t tick_long, uint32_t tick_verylong) {
  uint16_t v = get_data();

  if (v > threshold_upper && !pressed) {
    pressed = 1;
  } else if (v < threshold_lower && pressed) {
    pressed = 0;
  }

  if (!fn) return pressed;

  if (pressed) {
    // initial press
    if (!occupied) {
      occupied = 1;
      if (led) pins_flash(led, 1);
      start_tick = clock.current_tick;
    }

    uint32_t now = clock.current_tick;
    if (now-start_tick > tick_verylong) {
      type = VERYLONG;
    } else if (now-start_tick > tick_long) {
      type = LONG;
    } else {
      type = SHORT;
    }
  }

  // set/unset led depending on push type
  if (led) {
    if (type == LONG) pins_set(led, 1);
    else if (type == VERYLONG) pins_set(led, 0);
  }

  // released and processing
  if (!pressed && occupied) {
    pins_set(&pins_led, 0);

    // DF("press was: %s\n", type == SHORT ? "SHORT" : (type == LONG ? "LONG" : "VERYLONG"));
    // callback
    (*fn)(type, clock.current_tick-start_tick);

    occupied = 0;
    pressed = 0;
    type = NONE;
  }

  return pressed;
}
