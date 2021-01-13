#include <avr/interrupt.h>
#include "timer.h"
#include "uart.h"

TIMER* TIMER::timer_pointer;

ISR(TCA0_OVF_vect) {
  if (TIMER::timer_pointer->_rounds) {
    TIMER::timer_pointer->_rounds--;
    TCA0.SINGLE.PER = TIMER::timer_pointer->_rounds ? 0xFFFF : TIMER::timer_pointer->_last_round_ticks;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm;
  } else {
    TIMER::timer_pointer->_timeout = 1;
  }

  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // clear int flag
}

TIMER::TIMER(uint32_t mhz) {
  _mhz = mhz;
  init();
}

TIMER::TIMER() {
  _mhz = F_CPU;
  init();
}

void TIMER::init() {
  TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;
  timer_pointer = this;
}

/*
 * one round = 6.7s (= 1/(10MHz/1024/2^16)
 * max ms: 1711s = 28h 31min 16sec
 */
void TIMER::start(uint16_t ms) {
  _timeout = 0;
  _last_round_ticks = _mhz / 1000 * ms / 1024; // 10MHz/1024*2000ms/1000ms = 19532 (ensure no overflow by dividing by 1000 1st)
  _rounds = _last_round_ticks / 65535;

  if (_rounds) {
    _last_round_ticks = _last_round_ticks - _rounds*65535;
  }

  TCA0.SINGLE.CNT = 0;                                                     // ensure counter is 0 when starting
  TCA0.SINGLE.PER = _rounds ? 0xFFFF : _last_round_ticks;                  // 10MHz/1024/19532 = 0.5Hz (max 16bit)
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm; // prescaler 1024
  sei();
}

void TIMER::stop() {
  TCA0.SINGLE.CTRLA = 0;
}

uint8_t TIMER::timed_out() {
  return _timeout;
}
