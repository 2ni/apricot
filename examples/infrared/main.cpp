#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"

/*
 *
 * PA7: ir input
 * PA6: ir output
 * PC3-5: buttons
 *
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

namespace RECEIVE {
  typedef enum {
    NONE,
    STARTED,
    FINISHED,
    FAILED,
  } Type_Receive;
}

uint8_t edge = 0;
uint8_t cmd_in[4] = {0};
uint8_t cmd_in_bit_pos = 0;
uint8_t cmd_in_pos = 0;
volatile RECEIVE::Type_Receive  data_ready = RECEIVE::NONE;
volatile uint8_t data_sent = 0;

 /*
 * RAV231:
 * v+: 7a 85 1a e5
 * v-: 7a 85 1b e4
 * power on: 7a 85 1d e2
 * power off: 7a 85 1e e1
 */
const uint8_t commands[5][4] PROGMEM = {
  {0x86, 0x6b, 0x01, 0xfe}, // "1" vitaaudio
  {0x7a, 0x85, 0x1d, 0xe2}, // "power on" rav231
  {0x7a, 0x85, 0x1e, 0xe1}, // "power off" rav231
  {0x7a, 0x85, 0x1a, 0xe5}, // "vol up" rav231
  {0x7a, 0x85, 0x1b, 0xe4}  // "vol down" rav231
};

namespace CMD {
  typedef enum {
    ONE,
    PWR_ON,
    PWR_OFF,
    VOL_UP,
    VOL_DOWN,
  } Type_Cmd;
}

uint8_t buff_out[4] = {0};
uint8_t buff_out_pos = 0;
uint8_t buff_out_bit_pos = 0;
uint8_t buff_cur_bit = 0;
uint16_t cmd_out_cnt = 0;
uint16_t cmd_out_rep_cnt = 0;

FORMAT::Type_Format edge_out = FORMAT::NONE;

FORMAT::Type_Format format = FORMAT::NONE;

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
  format = FORMAT::NONE;
  PORTB.OUTCLR = PIN5_bm;
  if (data_ready == RECEIVE::STARTED) data_ready = RECEIVE::FAILED;
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
        switch (format) {
          case FORMAT::NONE:
            data_ready = RECEIVE::STARTED;
            start_timing();
            break;
          case FORMAT::NEC_FIRST:
            stop_timing();
            if (TCA0.SINGLE.CNT > 10000 && TCA0.SINGLE.CNT < 12500) { // 4.5ms = 11250 ticks
              format = FORMAT::NEC;
              for (uint8_t i=0; i<4; i++) cmd_in[i] = 0; // we have new data starting -> clear it
            }
            else if (TCA0.SINGLE.CNT > 5000 && TCA0.SINGLE.CNT < 6250) { // 2.25ms = 5625
              format = FORMAT::REPEAT;
              PORTB.OUTSET = PIN5_bm;
            }
            reset();
            break;
          case FORMAT::NEC:
            stop_timing();
            if (TCA0.SINGLE.CNT > 5000) {  // 2000us = 5000 ticks
              DF("* %u\n", TCA0.SINGLE.CNT);
            }
            // DF("b: %u\n", TCA0.SINGLE.CNT);
            cmd_in[cmd_in_pos] |= (TCA0.SINGLE.CNT > 2500) << cmd_in_bit_pos; // 1000us = 2500 ticks (let's be conservative)
            if ((cmd_in[cmd_in_pos] & cmd_in_bit_pos) == 1) PORTB.OUTSET = PIN5_bm; else PORTB.OUTCLR = PIN5_bm;
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
        switch (format) {
          case FORMAT::NONE:
            stop_timing();
            if (TCA0.SINGLE.CNT > 21000 && TCA0.SINGLE.CNT < 24000) { // 9ms = 22500 ticks
              format = FORMAT::NEC_FIRST;
              start_timing();
            }
            break;
          case FORMAT::NEC:
            if (cmd_in_pos == 4) {
              data_ready = RECEIVE::FINISHED;
              format = FORMAT::NONE;
              PORTB.OUTCLR = PIN5_bm;
              reset();
            } else {
              start_timing();
            }
            break;
          case FORMAT::REPEAT:
            data_ready = RECEIVE::FINISHED;
            PORTB.OUTCLR = PIN5_bm;
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
#define C_CARRIER 132 //(uint8_t)(10000.0/38/2+.5)           // 132  (37.878kHz)
#define C_PRE1    682 //(uint16_t)(9000.0*10/C_CARRIER+.5)   // 682
#define C_PRE2    341 //(uint16_t)(4500.0*10/C_CARRIER+.5)   // 341
#define C_LOW     43  //(uint16_t)(562.0*10/C_CARRIER+.5)    // 43
#define C_HIGH    127 //(uint16_t)(1675.0*10/C_CARRIER+.5)   // 127
#define C_REPW    8333 //(uint16_t)(110000.0*10/C_CARRIER+.5) // 8333
#define C_REP1    682 //(uint16_t)(9000.0*10/C_CARRIER+.5)   // 682
#define C_REP2    170 //(uint16_t)(2250.0*10/C_CARRIER+.5)   // 170
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;

  switch (edge_out) {
    case FORMAT::NEC_FIRST:
      if (cmd_out_cnt == 0) cmd_out_rep_cnt = 0;
      if (cmd_out_cnt < C_PRE1) PORTA.OUTTGL = PIN6_bm; // 9ms
      else {
        cmd_out_cnt = 0;
        PORTA.OUTCLR = PIN6_bm;
        edge_out = FORMAT::NEC;
      }
      break;
    case FORMAT::NEC:
      if (cmd_out_cnt >= C_PRE2) { // 4.5ms
        cmd_out_cnt = 0;
        edge_out = FORMAT::BIT_FIRST;
      }
      break;
    case FORMAT::BIT_FIRST:
      if (cmd_out_cnt < C_LOW) PORTA.OUTTGL = PIN6_bm;
      else {
        cmd_out_cnt = 0;
        PORTA.OUTCLR = PIN6_bm;
        edge_out = FORMAT::BIT_SECOND;
      }
      break;
    case FORMAT::BIT_SECOND:
      // we're done (last is just a half first bit)
      if (buff_out_pos == 4 && buff_out_bit_pos == 1) {
        buff_out_pos = 0;
        buff_out_bit_pos = 0;
        edge_out = FORMAT::REPEAT;
        cmd_out_cnt = 1000;
        data_sent = 1;
      }
      else if (cmd_out_cnt == 1) {
        buff_cur_bit = (buff_out[buff_out_pos] >> buff_out_bit_pos) & 0x01;
        if (++buff_out_bit_pos == 8) {
          buff_out_bit_pos = 0;
          buff_out_pos++;
        }
      }
      else if ((buff_cur_bit && cmd_out_cnt >= C_HIGH) || (!buff_cur_bit && cmd_out_cnt >= C_LOW)) { // 1 -> wait for 1675us, 0-> wait for 562us
        edge_out = FORMAT::BIT_FIRST;
        cmd_out_cnt = 0;
        PORTA.OUTCLR = PIN6_bm;
      }
      break;
    case FORMAT::REPEAT:
      if (cmd_out_rep_cnt == C_REPW) {
        cmd_out_rep_cnt = 0;
        cmd_out_cnt = 0;
      }

      if (cmd_out_cnt < C_REP1 || (cmd_out_cnt > (C_REP1+C_REP2) && cmd_out_cnt < (C_REP1+C_REP2+C_LOW))) {
        PORTA.OUTTGL = PIN6_bm;
      }
      else if (cmd_out_cnt == C_REP1) {
        PORTA.OUTCLR = PIN6_bm;
      }
      else if (cmd_out_cnt == (C_REP1+C_REP2+C_LOW)) {
        PORTA.OUTCLR = PIN6_bm;
        data_sent = 2;
      }
      break;
    default:
      break;
  }
  cmd_out_cnt++;
  cmd_out_rep_cnt++;
}

void init_timer() {
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc; // 10ms = .01/(1/10^6*4)
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;

  TCB0.CNT = 0;
  TCB0.CCMP = C_CARRIER; // 38k.023Hz
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // | TCB_ENABLE_bm;
  TCB0.INTCTRL = TCB_CAPT_bm;
}

void send_cmd(CMD::Type_Cmd cmd) {
  // load command to output buffer
  DF("sending: %s\n", cmd == CMD::PWR_ON ? "pwr on" : (cmd == CMD::PWR_OFF ? "pwr off" : (cmd == CMD::VOL_UP ? "vol up" : (cmd == CMD::VOL_DOWN ? "vol down" : "-"))));
  for (uint8_t i=0; i<4; i++) {
    buff_out[i] = pgm_read_byte(&(commands[cmd][i]));
  }
  cmd_out_cnt = 0;
  cmd_out_rep_cnt = 0;
  edge_out = FORMAT::NEC_FIRST;
  TCB0.CNT = 0;
  TCB0.CTRLA |= TCB_ENABLE_bm;
}

volatile int8_t pressed = 0;
ISR(PORTC_PORT_vect) {
  uint8_t flags = PORTC.INTFLAGS;
  PORTC.INTFLAGS = flags; // clear flags

  uint8_t new_press = 0;
  // to handle both edges (falling & rising)
  /*
  if (flags & PORT_INT3_bm) new_press = PORTC.IN & PIN3_bm ? -3 : 3;
  else if (flags & PORT_INT4_bm) new_press = PORTC.IN & PIN4_bm ? -4 : 4;
  else if (flags & PORT_INT5_bm) new_press = PORTC.IN & PIN5_bm ? -5 : 5;
  */

  if (flags & PORT_INT3_bm) new_press = 3;
  else if (flags & PORT_INT4_bm) new_press = 4;
  else if (flags & PORT_INT5_bm) new_press = 5;

  if (new_press != pressed) pressed = new_press;
}

int main(void) {
  uint32_t ts = 0;
  int8_t button = 0;
  uint8_t keep_awake = 0;
  CMD::Type_Cmd last_cmd = CMD::PWR_OFF;

  // deactivate any unused pin
  PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;

  // debug oscilloscope
  PORTB.DIRSET = PIN7_bm;

  // ir input signal
  PORTA.DIRCLR = PIN7_bm;
  PORTA.PIN7CTRL = PORT_ISC_BOTHEDGES_gc;
  PORTA.INTFLAGS = PORT_INT7_bm;

  // ir output signal
  PORTA.DIR |= PIN6_bm;
  PORTA.OUTCLR = PIN6_bm;
  PORTA.PIN6CTRL = PORT_ISC_INTDISABLE_gc;

  // button inputs (active low)
  PORTC.DIR &= ~(PIN5_bm | PIN4_bm | PIN3_bm);
  PORTC.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN4CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN5CTRL = PORT_PULLUPEN_bm;
  _delay_ms(1); // avoid triggering on start
  PORTC.PIN3CTRL |= PORT_ISC_FALLING_gc; // PORT_ISC_BOTHEDGES_gc
  PORTC.PIN4CTRL |= PORT_ISC_FALLING_gc;
  PORTC.PIN5CTRL |= PORT_ISC_FALLING_gc;


  mcu_init();
  init_timer();

  // ir receiver gets some dirty signals on startup
  _delay_ms(50);
  data_ready = RECEIVE::NONE;

  while (1) {
    // handle ir reception
    if (data_ready == RECEIVE::STARTED) {
      keep_awake |= (1<<1);
      while (data_ready != RECEIVE::FINISHED && data_ready != RECEIVE::FAILED);
      if (data_ready == RECEIVE::FINISHED) {
        uart_arr("in", cmd_in, 4);
        keep_awake &= ~(1<<1);
      }
      data_ready = RECEIVE::NONE;
    }

    // handle buttons (debounce)
    if (pressed && ts == 0) {
      ts = clock.current_tick;
    }
    else if (ts && (clock.current_tick - ts) > 205) { // 50ms = 50/8000*32768 = 204.8
      if ((PORTC.IN & (1<<pressed)) == 0 && !button) {
        button = pressed;
        keep_awake |= (1<<0);
        PORTB.OUTSET = PIN5_bm;
        switch (button) {
          case 3:
            last_cmd = last_cmd == CMD::PWR_ON ? CMD::PWR_OFF : CMD::PWR_ON;
            send_cmd(last_cmd);
            break;
          case 4:
            send_cmd(CMD::VOL_UP);
            break;
          case 5:
            send_cmd(CMD::VOL_DOWN);
            break;
        }
      } else {
        pressed = 0;
        ts = 0;
      }
    }

    // button released
    if (button && (PORTC.IN & (PIN5_bm | PIN4_bm | PIN3_bm)) == (PIN5_bm | PIN4_bm | PIN3_bm)) {
      TCB0.CTRLA &= ~TCB_ENABLE_bm;
      PORTB.OUTCLR = PIN5_bm;
      keep_awake &= ~(1<<0);
      pressed = 0;
      button = 0;
      ts = 0;
    }

    /*
    if (data_sent == 1) {
      DL("sent");
      data_sent = 0;
    }
    else if (data_sent == 2) {
      DL("rep");
      data_sent = 0;
    }
    */

    // RTC will wake up, so we need to go sleep in each loop
    if (!keep_awake) {
      while (uart_is_busy());
      PORTB.OUTSET = PIN7_bm;
      __asm__ __volatile__ ( "sleep" "\n\t" :: );
      PORTB.OUTCLR = PIN7_bm;
    }
  }
}
