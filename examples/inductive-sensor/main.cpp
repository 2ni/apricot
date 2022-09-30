/*
 * inductive sensor on PA6 (AINP0, AC2)
 * set AINN0 to Vref (0.55v)
 *
 * max: 200kHz, min 4mV diff from 0.55v
 *
 * give a short pulse
 * set high impedance (input)
 * interrupt generation on rising edge
 * set hysteresis (10mV, 25mV or 50mV)
 *
 *  +3.3v ---+
 *           |  +-C1-+
 *          R1  |    |
 *           |--+-L1-+-R3--> PA6
 *           |  |
 *          R2 C2
 *           |  |
 *  GND -----+--+
 *
 *  R1=100k, R2=20k, L1=3.3u, C1=100n, R3=50
 *
 * partly based on http://coole-basteleien.de/naeherungssensor
 * we use
 */
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

// #define wait1us asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n")  // 1us
#define _wait() do { __asm__ __volatile__ ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"); } while (0)
#define PIN_SIGNAL PIN6_bm
#define SET_HIGH() (PORTA.OUTSET = PIN_SIGNAL)
#define SET_LOW() (PORTA.OUTCLR = PIN_SIGNAL)
#define SET_INPUT() (PORTA.DIRCLR = PIN_SIGNAL)
#define SET_OUTPUT() (PORTA.DIRSET = PIN_SIGNAL)

volatile uint16_t counter = 0;
volatile uint16_t counter_fetch = 0;
volatile uint8_t measure_done = 0;

ISR(AC2_AC_vect) {
  AC2.STATUS = AC_CMP_bm; // clear int flag
  counter++;
}

/*
 * DV64 -> measure 6.4ms every 1sec (6.4*156)
 */
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  counter_fetch++;
  if (counter_fetch == 1) {
    // end measure
    AC2.CTRLA &= ~AC_ENABLE_bm;
    measure_done = 1;
    PORTA.OUTCLR = PIN5_bm;
  } else if (counter_fetch == 156) { // DIV1: 10000, DIV64: 156
    PORTA.OUTSET = PIN5_bm;
    counter_fetch = 0;
    // start measure
    AC2.CTRLA |= AC_ENABLE_bm;
  }
}

int main(void) {
  mcu_init();

  // set INTMODE in AC2.CTRLA, AC2.INTCTRL
  // LPMODE off (no low power mode for now)
  // hysteresis: AC_HYSMODE_OFF_gc or AC_HYSMODE_10mV_gc
  VREF.CTRLD = VREF_DAC2REFSEL_0V55_gc; // set DAC2 to 0.55 (also for AC2)
  AC2.CTRLA = AC_INTMODE_POSEDGE_gc | AC_LPMODE_DIS_gc | AC_HYSMODE_OFF_gc; // | AC_ENABLE_bm;
  AC2.MUXCTRLA = AC_MUXPOS_PIN0_gc | AC_MUXNEG_VREF_gc;

  // PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;

  AC2.INTCTRL = AC_CMP_bm;
  AC2.CTRLA |= 1; // enable interrupts

  PORTA.DIRSET = PIN5_bm;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.PER = 1000;
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV64_gc;
  PORTA.OUTSET = PIN5_bm;

  while (1) {
    if (measure_done) {
      DF("frq : %ukHz\n", (counter*10)>>6); // DIV1: *10, DIV64: *10>>6
      counter = 0;
      measure_done = 0;
    }
  }

  /*
  while (1) {
    SET_OUTPUT();
    SET_HIGH();
    _wait();
    SET_LOW();
    SET_INPUT();

    AC2.INTCTRL = AC_CMP_bm;
    _delay_us(15);
    AC2.INTCTRL = 0;
    while(1);
  }
  */
}
