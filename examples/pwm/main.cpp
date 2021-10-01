#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"

/*
 * https://www.avrfreaks.net/forum/tut-attiny817-12-bit-timer-tcd-servos-1-2-ramp-pwm-hxt900
 * TCD0 one ramp mode page 273+
 * WOA = PA4
 * WOB = PA5
 * CMPBCLR: defines frequency 0-2^12
 * frequency: f / (CMPBCLR + 1)
 * CMPASET = 0
 * CMPACLR = 0-CMPBCLR
 */

typedef enum {
  NONE,
  FREQUENCY,
  PWM,
} Selected_element_type;

Selected_element_type selected_element = NONE;
uint16_t value = 0;
char input[5];
uint8_t input_len = 0;
uint16_t frequency = 39;
uint16_t pwm = 25;

void update(uint16_t frequency, uint16_t pwm) {
  TCD0.CMPBCLR = 20000/frequency;
  TCD0.CMPBSET = TCD0.CMPBCLR - TCD0.CMPBCLR*pwm/100;
  while (!(TCD0.STATUS & TCD_ENRDY_bm)); // wait for any synch going on
  TCD0.CTRLE = TCD_SYNC_bm;
  DF("frq: %u, pwm: %u, steps: %u (%u/%u)\n", frequency, pwm, TCD0.CMPBCLR, TCD0.CMPBSET, TCD0.CMPBCLR);
}

ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  if (selected_element == NONE) {
    switch (in) {
      case 'f':
        D("frq >");
        selected_element = FREQUENCY;
        break;
      case 'p':
        D("pwm >");
        selected_element = PWM;
        break;
      case '+':
        pins_flash(&pins_led, 3, 100);
        break;
    }
    // clear input
    memset(input, 0, 5);
    input_len = 0;
  } else {
    if (in == '\n') {
      pins_flash(&pins_led, 1, 100);
      value = atoi(input);
      if (selected_element == FREQUENCY) frequency = value;
      else if (selected_element == PWM) pwm = value;
      update(frequency, pwm);
      selected_element = NONE;
    } else {
      input[input_len] = in;
      input_len++;
    }
  }
}

int main(void) {
  mcu_init(1); // init uart read
  pins_output(&PA5, 1); // set pwm output

  DL("Use f for frequency in khz\np for pwm in 0-100%");

  TCD0.CTRLA = TCD_CNTPRES_DIV1_gc | TCD_SYNCPRES_DIV1_gc | TCD_CLKSEL_20MHZ_gc;
  TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;

  _PROTECTED_WRITE(TCD0.FAULTCTRL, TCD_CMPAEN_bm | TCD_CMPBEN_bm);
  TCD0.CMPASET = 0;
  update(frequency, pwm);

  TCD0.CTRLA |= TCD_ENABLE_bm;

  while (1);
}
