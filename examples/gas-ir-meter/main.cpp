/*
 *
 *  +3.3v ---+--+
 *           |  |
 *          R3  R2
 *        ir >  |
 *  PA4--R1--   +---PA5
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
 *
 * to not miss any measurement, we use a buffer which is filled by the ISR
 * and processed during main loop
 *
 * when the IR is activate it needs ~200-400us to settle on the correct value
 * this is why we use the initialization delay of the adc to avoid _delay's in the ISR
 * (ISR's don't like _delay's)
 *
 */
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

#define MEAS_LEN 4 // number of measurements, should be 2,4,8,16,...
uint8_t measurements[MEAS_LEN] = {0};
uint8_t p_m = 0; // pointer of measurements
uint8_t limit = 0; // limit for an active measurement (below the limi we have a match)
#define THRESHOLD 10 // threshold which triggers an active measurement
uint8_t active = 0;

#define BUFFER_LEN 16
volatile uint8_t buffer[BUFFER_LEN] = {0};
volatile uint8_t buffer_pw = 0;
uint8_t buffer_pr = 0;

#define CALIBRATION_TICKS 20
volatile uint8_t c_ticks = CALIBRATION_TICKS; // depends on RTC.PITCTRLA, eg RTC_PERIOD_CYC8192_gc: 20*8192/32768 = 5sec
#define RTC_PERIOD  13 // 2^13/32768 = 250ms
#define RTC_PERIOD_CYC_gc (RTC_PERIOD-1)<<3 // see eg RTC_PERDIO_CYC8192
#define CALIBRATION_DURATION (uint32_t)(1<<RTC_PERIOD)*CALIBRATION_TICKS/32768
#define TICKS_PER_SECOND (uint8_t)(32768/(1<<RTC_PERIOD))

#define _sleep() do { __asm__ __volatile__ ( "sleep" "\n\t" :: ); } while (0)

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  if (c_ticks) c_ticks--;

  PORTA.OUTSET = PIN4_bm; // activate ir transmitter
  ADC0.CTRLA |= ADC_RUNSTBY_bm | ADC_ENABLE_bm;
  ADC0.COMMAND = 1;
}

ISR(ADC0_RESRDY_vect) {
  PORTA.OUTCLR = PIN4_bm; // disable ir
  buffer[buffer_pw] = ADC0.RES;
  ADC0.CTRLA &= ~(ADC_RUNSTBY_bm | ADC_ENABLE_bm);
  buffer_pw = (buffer_pw+1) % BUFFER_LEN;
}

int main(void) {
  mcu_init(0, 0); // disable RTC, we do our own

  SLPCTRL.CTRLA = SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm; // SLPCTRL_SMODE_PDOWN_gc, SLPCTRL_SMODE_STDBY_gc

  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
  // RTC.PER = 32; // 33/32768=1.00708008ms
  // RTC.INTCTRL = RTC_OVF_bm;
  // RTC.CNT = 0;

  RTC.PITINTCTRL = RTC_PI_bm; // enable PIT
  RTC.PITCTRLA = RTC_PERIOD_CYC_gc | RTC_PITEN_bm; // RTC_PERIOD_CYC_gc is set as eg RTC_PERDIO_CYC8192
  while (RTC.STATUS > 0);
  sei();

  PORTA.DIRSET = PIN4_bm; // ir transmitter
  PORTA.OUTCLR = PIN4_bm;
  PORTA.DIRCLR = PIN5_bm; // ir receiver
  PORTA.DIRSET = PIN3_bm; // oscilloscope debug

  VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
  ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp); // ADC_REFSEL_INTREF_gc, ADC_REFSEL_VDDREF_gc
  ADC0.MUXPOS = ADC_MUXPOS_AIN5_gc;
  ADC0.CTRLD = ADC_INITDLY_DLY256_gc; // delay measurement by 1/(10MHz/2)*256 = 409us
  ADC0.CTRLA = ADC_RESSEL_bm;
  ADC0.INTCTRL = ADC_RESRDY_bm;

  // calibration: we just fill the array measurement for a certain time
  DF("calibrating for %lus, ticks/s: %u\n", CALIBRATION_DURATION, TICKS_PER_SECOND);
  while (c_ticks) {
    PORTA.OUTSET = PIN4_bm;
    ADC0.CTRLA |= ADC_ENABLE_bm;
    ADC0.COMMAND = 1;
    uint8_t buffer_pw_temp = buffer_pw;
    while (buffer_pr != buffer_pw_temp) {
      DF("%u: %u\n", buffer_pr, buffer[buffer_pr]);
      measurements[p_m] = buffer[buffer_pr];
      p_m = (p_m+1) % MEAS_LEN;
      buffer_pr = (buffer_pr+1) % BUFFER_LEN;
    }

    if (c_ticks < TICKS_PER_SECOND) {
      PORTB.OUTSET = PIN5_bm;
    }
    while (uart_is_busy());
    _sleep();
  }
  PORTB.OUTCLR = PIN5_bm;
  uint16_t sum = 0;
  for (uint8_t i=0; i<MEAS_LEN; i++) {
    sum += measurements[i];
  }
  limit = sum/MEAS_LEN - THRESHOLD;
  DF("calibration done. limit: %u\n****************\n\n", limit);

  /*
   * auto calibration
   * only take values if more or less stable and not in active mode
   * if sum of differences too high -> mode = recalibrate, wait for stable data
   */
  while (1) {
    // process new incoming measurements from the buffer
    uint8_t buffer_pw_temp = buffer_pw;
    if (buffer_pr != buffer_pw_temp) {
      while (buffer_pr != buffer_pw_temp) {
        DF("%u (%u)\n", buffer[buffer_pr], limit);

        if (!active && buffer[buffer_pr] < limit) {
          active = 1;
          DL("active!");
          PORTB.OUTSET = PIN5_bm;
          _delay_ms(20);
          PORTB.OUTCLR = PIN5_bm;
        }

        if (buffer[buffer_pr] >= limit) {
          measurements[p_m] = buffer[buffer_pr];
          p_m = (p_m+1) % MEAS_LEN;

          uint16_t sum = 0;
          for (uint8_t i=0; i<MEAS_LEN; i++) {
            sum += measurements[i];
          }
          limit = sum/MEAS_LEN - THRESHOLD;

          if (active) {
            active = 0;
            DL("inactive");
          }
        }

        buffer_pr = (buffer_pr+1) % BUFFER_LEN;
      }
    }

    // do some busy stuff (ensure the buffer is large enough, eg BUFFER_LEN 16, measure frq: 1/4Hz -> max 4s
    // _delay_ms(3000);

    // TODO if we're not measuring ADC we could go deep sleep -> introduce status variable
    while (uart_is_busy());
    _sleep();
  }
}
