/*
 *
 *  +3.3v ---+--+
 *           |  |
 *          R3  R2
 *        ir >  |
 *  PA3--R1--   +---PA5
 *           |  |
 *           |  < ir-REC
 *           |  |
 *  GND -----+--+
 *
 *  R1,R3=100, R2=22k, Rgate=100
 *  PA5=AIN5 (ADC0)
 *
 * default measurement: ~240
 * active measurement:  ~215-225 (~4 values if 0.5sec intervall)
 */
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

#define NUM_OF_MEAS 4 // number of measurements, should be 2,4,8,16,...
uint8_t measurements[NUM_OF_MEAS] = {0};
uint8_t current_meas = 0;
uint8_t got_meas = 0; // true if we have an active value
uint8_t p_m = 0; // pointer of measurements
uint8_t initalizing = 30; // number of ticks, the initialisation lasts
uint8_t init_done = 0;

#define _sleep() do { __asm__ __volatile__ ( "sleep" "\n\t" :: ); } while (0)

volatile uint8_t work = 0;
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  if (initalizing) initalizing--;
  work = 1;
}

int main(void) {
  mcu_init(0, 0); // disable RTC, we do our own

  SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm; // SLPCTRL_SMODE_PDOWN_gc, SLPCTRL_SMODE_STDBY_gc

  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
  // RTC.PER = 32; // 33/32768=1.00708008ms
  // RTC.INTCTRL = RTC_OVF_bm;
  // RTC.CNT = 0;

  RTC.PITINTCTRL = RTC_PI_bm; // enable PIT
  RTC.PITCTRLA = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;
  while (RTC.STATUS > 0);
  sei();

  PORTA.DIRSET = PIN3_bm; // ir transmitter
  PORTA.OUTCLR = PIN3_bm;
  PORTA.DIRCLR = PIN5_bm; // ir receiver

  VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
  ADC0.CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp); // ADC_REFSEL_INTREF_gc, ADC_REFSEL_VDDREF_gc
  ADC0.MUXPOS = ADC_MUXPOS_AIN5_gc;
  ADC0.CTRLA = ADC_RESSEL_bm | ADC_ENABLE_bm;

  // initalization
  ADC0.COMMAND = 1;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  uint8_t measurement = ADC0.RES;
  for (uint8_t i=0; i<NUM_OF_MEAS; i++) {
    measurements[i] = measurement;
  }

  DF("initializing for: %us\n", initalizing/2); // 32768/RTC_PERIOD_CYC
  while (1) {
    if (work) {
      work = 0;
      PORTA.OUTSET = PIN3_bm; // activate ir transmitter
      _delay_ms(1); // wait for ir to settle up
      ADC0.COMMAND = 1;
      while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
      current_meas = ADC0.RES;
      PORTA.OUTCLR = PIN3_bm; // disable ir

      uint16_t sum = 0;
      for (uint8_t i=0; i<NUM_OF_MEAS; i++) {
        sum += measurements[i];
      }
      uint8_t limit = sum/NUM_OF_MEAS - 10;
      DF("%u (%u)\n", current_meas, limit);

      if (!initalizing) {
        if (!init_done) {
          DL("init done");
          init_done = 1;
        }

        // if drop is larger than average of last measurements -> we've got a match
        if (!got_meas && current_meas < limit) {
          got_meas = 1;
          PORTB.OUTSET = PIN5_bm;
          _delay_ms(20);
          PORTB.OUTCLR = PIN5_bm;
        }

        if (got_meas && current_meas >= limit) {
          DL("released");
          got_meas = 0;
        }
      }

      if (initalizing || !got_meas) {
        measurements[p_m] = current_meas;
        p_m = (p_m+1) % NUM_OF_MEAS;
      }
      while (uart_is_busy());
      _sleep();
    }
  }
}
