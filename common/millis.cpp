/*
 *
 * provide a reliable timer which is constantly running
 *
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

volatile uint32_t millis;

// TCB overflow handler, called every millisecond.
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm; // clear interrupt flag
  millis++;
}

void millis_init(uint32_t cpu) {
  TCB0.CNT = 0;                 // ensure counter starts at 0
  TCB0.CCMP = cpu / 1000 - 1;
  TCB0.CTRLA = /*TCB_RUNSTDBY_bm | */TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  TCB0.CTRLB = 0; // periodic interrupt
  TCB0.INTCTRL = TCB_CAPT_bm;

  sei();
}

void millis_init() {
  millis_init(F_CPU);
}

uint32_t millis_time() {
  cli();
  uint32_t r = millis;
  sei();
  return r;
}
