#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"

/*
 *
 * PA7: ir input
 * PA6: ir output
 *
 *
 * RAV231:
 * v+: 7a 85 1a e5
 * v-: 7a 85 1b e4
 * power on: 7a 85 1d e2
 * power off: 7a 85 1e e1
 */
namespace FORMAT {
  typedef enum {
    NONE,
    REPEAT,
    NEC_FIRST,
    NEC,
    BIT_FIRST,
    BIT_SECOND,
  } Type_Format;
}
volatile uint8_t edge = 0;
uint8_t cmd_in[4] = {0};
volatile uint8_t cmd_in_bit_pos = 0;
volatile uint8_t cmd_in_pos = 0;
volatile uint8_t data_ready = 0;
volatile uint8_t data_sent = 0;

uint8_t cmd_out[4] = {0x86, 0x6b, 0x01, 0xfe};
volatile FORMAT::Type_Format edge_out = FORMAT::NONE;
volatile uint16_t cmd_out_cnt = 0;
volatile uint16_t cmd_out_rep_cnt = 0;
volatile uint8_t cur_bit = 0;
volatile uint8_t cmd_out_pos = 0;
volatile uint8_t cmd_out_bit_pos = 0;

volatile FORMAT::Type_Format format = FORMAT::NONE;

void reset() {
  cmd_in_bit_pos = 0;
  cmd_in_pos = 0;
}

void start_timing() {
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

void stop_timing() {
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}

ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
  reset();
  DL("ovf");
}

/*
 * interrupt on port change
 * ir receiver
 * with DIV4: 0: 562us  = 1405 ticks (= 562uS*10MHz/4)
 *            1: 1675us = 4188 ticks
 */
ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // clear flags

  if (flags & PORT_INT7_bm) {
    switch (edge) {
      case 0:
        switch(format) {
          case FORMAT::NONE:
            start_timing();
            break;
          case FORMAT::NEC_FIRST:
            stop_timing();
            if (TCA0.SINGLE.CNT > 10000 && TCA0.SINGLE.CNT < 12500) { // 4.5ms = 11250 ticks
              format = FORMAT::NEC;
              for (uint8_t i=0; i<4; i++) cmd_in[i] = 0; // we have new data starting -> clear it
              data_ready = 0;
            }
            else if (TCA0.SINGLE.CNT > 5375 && TCA0.SINGLE.CNT < 5875) format = FORMAT::REPEAT; // 2.25ms = 5625
            reset();
            break;
          case FORMAT::NEC:
            stop_timing();
            if (TCA0.SINGLE.CNT > 5000) {  // 2000us = 5000 ticks
              DF("* %u\n", TCA0.SINGLE.CNT);
            }
            // DF("b: %u\n", TCA0.SINGLE.CNT);
            cmd_in[cmd_in_pos] |= (TCA0.SINGLE.CNT > 2500) << cmd_in_bit_pos; // 1000us = 2500 ticks (let's be conservative)
            if (++cmd_in_bit_pos == 8) {
              cmd_in_bit_pos = 0;
              cmd_in_pos++;
            }
            break;
          default:
            break;
        }
        break;
      case 1:
        switch(format) {
          case FORMAT::NONE:
            stop_timing();
            if (TCA0.SINGLE.CNT > 21000 && TCA0.SINGLE.CNT < 24000) { // 9ms = 22500 ticks
              format = FORMAT::NEC_FIRST;
              start_timing();
            }
            break;
          case FORMAT::NEC:
            if (cmd_in_pos == 4) {
              data_ready = 1;
              format = FORMAT::NONE;
              reset();
            } else {
              start_timing();
            }
            break;
          case FORMAT::REPEAT:
            data_ready = 1;
            format = FORMAT::NONE;
            reset();
            break;
          default:
            start_timing();
            break;
        }
        break;
    }
    edge = (edge + 1) % 2;
  }
}

/*
 * transmitter
 * 38kHz carrier with TCB (CCB=10MHz/38kHz/2=131.6)
 * counts = duration * (10MHz/CCB)
 * preamble: 9ms (682 counts) low,  4.5ms (341 counts) high
 * 0: 562us (43 counts) low, 562us (43 counts) low
 * 1: 562us (43 counts) low, 1675us (127 counts) high
 * repeat 110ms (8333 counts) from start
 *   9ms (682 counts) low, 2.25ms (170 counts) high, 562us (43 counts) low
 */
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;

  switch (edge_out) {
    case FORMAT::NEC_FIRST:
      if (cmd_out_cnt < 682) PORTA.OUTTGL = PIN6_bm; // 9ms
      else {
        cmd_out_cnt = 0;
        edge_out = FORMAT::NEC;
      }
      break;
    case FORMAT::NEC:
      if (cmd_out_cnt >= 341) { // 4.5ms
        cmd_out_cnt = 0;
        edge_out = FORMAT::BIT_FIRST;
      }
      break;
    case FORMAT::BIT_FIRST:
      if (cmd_out_cnt <= 42) PORTA.OUTTGL = PIN6_bm;
      else {
        cmd_out_cnt = 0;
        edge_out = FORMAT::BIT_SECOND;
      }
      break;
    case FORMAT::BIT_SECOND:
      if (cmd_out_pos == 4 && cmd_out_bit_pos == 1) {
        cmd_out_pos = 0;
        edge_out = FORMAT::NONE;
        data_sent = 1;
      }
      else if (cmd_out_cnt == 1) {
        cur_bit = (cmd_out[cmd_out_pos] >> cmd_out_bit_pos) & 0x01;
        if (++cmd_out_bit_pos == 8) {
          cmd_out_bit_pos = 0;
          cmd_out_pos++;
        }
      }
      else if ((cur_bit && cmd_out_cnt >= 127) || (!cur_bit && cmd_out_cnt >= 43)) { // 1 -> wait for 1675us, 0-> wait for 562us
        edge_out = FORMAT::BIT_FIRST;
        cmd_out_cnt = 0;
      }
      break;
    default:
      break;
  }
  cmd_out_cnt++;
}

void init_timer() {
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc; // 10ms = .01/(1/10^6*4)
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;

  TCB0.CNT = 0;
  TCB0.CCMP = 130; // 38k.023Hz
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // | TCB_ENABLE_bm;
  TCB0.INTCTRL = TCB_CAPT_bm;
}

int main(void) {
  mcu_init();
  pins_output(&PA7, 0); // ir signal input
  PORTA.PIN7CTRL |= PORT_ISC_BOTHEDGES_gc;

  // ir output signal
  PORTA.DIR |= PIN6_bm;
  PORTA.OUT |= PIN6_bm;
  pins_set(&PA6, 1);

  init_timer();

  edge_out = FORMAT::NEC_FIRST;
  TCB0.CTRLA |= TCB_ENABLE_bm;

  while (1) {
    if (data_ready) {
      data_ready = 0;
      uart_arr("cmd_i", cmd_in, 4);
    }

    if (data_sent) {
      data_sent = 0;
      TCB0.CTRLA &= ~TCB_ENABLE_bm;
      uart_arr("cmd_o", cmd_out, 4);
    }
  }
}
