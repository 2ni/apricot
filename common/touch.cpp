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
  /*
  DF("touch threshold: %u\n", threshold);
  DF("touch threshold upper: %u\n", threshold_upper);
  DF("touch threshold lower: %u\n", threshold_lower);
  */
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

/*
 * returns statuses of pressed button
 * if led given, it'll flash when the button is initially pressed
 * TODO MCU can't sleep while pressed as it must measure time -> continue millis while sleeping
 *
 * MSB is set, if button is pressed
 * lower bits are set depending if short, long, very long push
 *
 * example of returned values:
 * 0x00  button not pushed
 * 0x81  button initially pushed, 1 says we're currently in modus "short"
 * 0x82  button still pushed, but 2 says we're currently in modus "long"
 * 0x02  button was released as long push
 * 0x00
 *
 * to check if button pressed:
 *   if button.is_pressed() {
 *     // do some stuff while button is pressed
 *   }
 * to check status while pressed:
 *   s = button.is_pressed();
 *   if (s & touch_is_pressed_bm) {
 *     if (status & ~touch_is_pressed_bm == touch_short_bm) {
 *       // do some stuff while button is pressed and detected as short push for now
 *     }
 *   }
 * to check value of released button:
 *   s = button.is_pressed();
 *   if (status && (~status & touch_is_pressed_bm)) {
 *     if (status == touch_long_bm) {
 *       // do some stuff when button was released as long push
 *     }
 *   }
 */
uint8_t TOUCH::is_pressed(pins_t *led) {
  uint16_t v = get_data();
  uint8_t s = 0;
  uint8_t just_released = 0;

  // button initially pushed (start)
  if (v > threshold_upper && !finger_present) {
    finger_present = 1;
    now = millis_time();
    if (led) pins_flash(led, 1);
  }
  // button released
  else if (finger_present && v < threshold_lower) {
    finger_present = 0;
    just_released = 1;
  }

  if (finger_present || just_released) {
    uint32_t n = millis_time() - now;
    if (n < 2000) s = touch_short_bm;
    else if (n < 5000) s = touch_long_bm;
    else s = touch_verylong_bm;
  }

  return (just_released ? 0x00 : (finger_present << 7)) | s;
}
