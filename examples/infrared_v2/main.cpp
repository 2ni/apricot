/*
 * make flash port=3 clk=5000000
 *
 * needs to run at 5MHz, as longest period is 13.5ms
 * TCB with CLKDIV2 can then measure up to 2^16*(1/(5000000/2)) = 26.2ms
 *
 * PA7: infrared input (uses TCB1)
 * PA5: infrared output (uses TCB0)
 * PC3-5: input buttons
 *
 */
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"

const uint32_t commands[61] PROGMEM = {
              // ********* vitaaudio
  0x866b01fe, // 1, menu
  0x866b02fd, // 2, down
  0x866b03fc, // 3, shuffle
  0x866b04fb, // 4, up
  0x866b05fa, // 5, enter
  0x866b06f9, // info
  0x866b12ed, // clock
  0x866b1ae5, // audio
  0x866b1ee1, // power
  0x866b08f7, // ok/play/pause
  0x866b07f8, // back/rewind
  0x866b09f6, // forward
  0x866b0af5, // dab
  0x866b1be4, // fm
  0x866b0cf3, // aux
  0x866b0df2, // ipod
  0x866b1fe0, // vol up
  0x866b0ef1, // vol down
              // ********* rav231
  0x7a851de2, // power on
  0x7a851ee1, // power off
  0x7a851ae5, // vol up
  0x7a851be4, // vol down
  0x7a851ce3, // mute
  0x7a858778, // 6ch input
  0x7a8514eb, // phono
  0x7a8516e9, // tuner
  0x7a8515ea, // cd
  0x7a858554, // v-aux
  0x7a85c03f, // cbl/sat
  0x7a8518e7, // md/tape
  0x7a8519e6, // cd-r
  0x7a8554ab, // d-tv/ld
  0x7a850ff0, // vcr1
  0x7a8513ec, // vcr2
  0x7a85c13e, // dvd
  0x7a8510ef, // tuner ch+
  0x7a8510ef, // tuner ch-
  0x7a85e51a, // tuner 1
  0x7a85e619, // tuner 2
  0x7a85e718, // tuner 3
  0x7a85e817, // tuner 4
  0x7a85e916, // tuner 5
  0x7a85ea15, // tuner 6
  0x7a85eb14, // tuner 7
  0x7a85ec13, // tuner 8
  0x7a85ed12, // tuner 9

  0x7a8556a9, // stereo
  0x7a858877, // hall
  0x7a858976, // church
  0x7a858a75, // jazz club
  0x7a858b74, // rock concert
  0x7a858c73, // entertainment
  0x7a858d72, // tv sports
  0x7a858e71, // mono movie
  0x7a858f70, // movie theater 1
  0x7a85906f, // movie theater 2
  0x7a85916e, // DTS
  0x7a859669, // DTS select
  0x9a0a2fd1, // 6.1/ES
  0x7a8552ad, // amp ch+
  0x7a8553ac, // amp ch-
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

uint32_t cmd_in = 0;
uint8_t p_cmd_in = 0;
volatile uint8_t cmd_received = 0; // 0: waiting for preamble, 1: receiving data, 2: receive done, 3: repeat
uint32_t ts_last_reception = 0;

/*
 * infrared input detection
 */
ISR(TCB1_INT_vect) {
  TCB1.INTFLAGS = TCB_CAPT_bm;

  // nec repeat: 11.25ms = 28125 ticks
  // repeat cmd is only valid if we had a cmd within the last ~140ms = 9 rtc ticks * (512 / 32768)
  if (TCB1.CCMP > 27375 && TCB1.CCMP < 28875 && (clock.current_tick - ts_last_reception) < 9) {
    cmd_received = 3; // repeat
    ts_last_reception = clock.current_tick;
  }
  else if (!cmd_received && TCB1.CCMP > 33000 && TCB1.CCMP < 34500) { // nec preamble: 13.5ms = 33750 ticks
    cmd_in = 0;
    p_cmd_in = 0;
    cmd_received = 1; // capture started
    ts_last_reception = clock.current_tick;
  }
  else if (cmd_received == 1) {
    // logic 1
    if (TCB1.CCMP > 5388 && TCB1.CCMP < 5788) { // 2*560u = 2800 ticks
      int8_t offset = p_cmd_in<8 ? 24 : (p_cmd_in<16 ? 8 : (p_cmd_in<24 ? -8 : -24));
      cmd_in |= (uint32_t)1<<(p_cmd_in+offset);
    }
    if (++p_cmd_in == 32) {
      cmd_received = 2; // capture done
    }
  }
}

namespace STATUS_SEND {
 typedef enum {
    PREAMBLE_POS,
    PREAMBLE_NEG,
    BIT_POS,
    BIT_NEG,
    BIT_LAST_POS,
    BIT_LAST_NEG,
    REPEAT_POS,
    REPEAT_NEG
  } Type_Status_Send;
}

// infrared output called every 562us
uint16_t out_cnt = 0;
uint16_t out_cnt_next = 0;
uint8_t cmd_out_p = 0;
STATUS_SEND::Type_Status_Send status_send = STATUS_SEND::PREAMBLE_POS;
uint32_t cmd_out = 0;
volatile uint8_t cmd_sent = 0;
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  switch (status_send) {
    case STATUS_SEND::PREAMBLE_POS:
      TCB0.CTRLA |= TCB_ENABLE_bm;
      out_cnt = 0;
      out_cnt_next = 16; // 9ms
      status_send = STATUS_SEND::PREAMBLE_NEG;
      cmd_out_p = 0;
      cmd_sent = 0;
      // PORTB.OUTSET = PIN7_bm;
      break;
    case STATUS_SEND::PREAMBLE_NEG:
      if (out_cnt >= out_cnt_next) {
        TCB0.CTRLA &= ~TCB_ENABLE_bm;
        out_cnt_next += 8; // 4.5ms
        status_send = STATUS_SEND::BIT_POS;
        // PORTB.OUTCLR = PIN7_bm;
      }
      break;
    case STATUS_SEND::BIT_POS:
      if (out_cnt >= out_cnt_next) {
        TCB0.CTRLA |= TCB_ENABLE_bm;
        out_cnt_next += 1; // 562us
        status_send = STATUS_SEND::BIT_NEG;
        // PORTB.OUTSET = PIN7_bm;
      }
      break;
    case STATUS_SEND::BIT_NEG:
      {
        if (out_cnt >= out_cnt_next) {
          TCB0.CTRLA &= ~TCB_ENABLE_bm;
          int8_t offset = cmd_out_p<8 ? 24 : (cmd_out_p<16 ? 8 : (cmd_out_p<24 ? -8 : -24));
          uint8_t bit = cmd_out & ((uint32_t)1<<(cmd_out_p+offset)) ? 1 : 0;
          out_cnt_next += bit ? 3 : 1; // 0: 562us, 1: 1686us
          if (++cmd_out_p == 32) {
            status_send = STATUS_SEND::BIT_LAST_POS;
          } else {
            status_send = STATUS_SEND::BIT_POS;
          }
          // PORTB.OUTCLR = PIN7_bm;
        }
      }
        break;
    case STATUS_SEND::BIT_LAST_POS:
        if (out_cnt >= out_cnt_next) {
          TCB0.CTRLA |= TCB_ENABLE_bm;
          out_cnt_next += 1;
          status_send = STATUS_SEND::BIT_LAST_NEG;
          // PORTB.OUTSET = PIN7_bm;
        }
        break;
    case STATUS_SEND::BIT_LAST_NEG:
        if (out_cnt >= out_cnt_next) {
          TCB0.CTRLA &= ~TCB_ENABLE_bm;
          out_cnt_next = 196; // 110ms
          cmd_sent = 1;
          status_send = STATUS_SEND::REPEAT_POS;
          // PORTB.OUTCLR = PIN7_bm;
        }
        break;
    case STATUS_SEND::REPEAT_POS:
        if (out_cnt >= out_cnt_next) {
          TCB0.CTRLA |= TCB_ENABLE_bm;
          out_cnt = 0;
          out_cnt_next = 16; // 9ms
          status_send = STATUS_SEND::REPEAT_NEG;
          // PORTB.OUTSET = PIN7_bm;
        }
        break;
    case STATUS_SEND::REPEAT_NEG:
        if (out_cnt >= out_cnt_next) {
          TCB0.CTRLA &= ~TCB_ENABLE_bm;
          out_cnt_next += 4; // 2.25ms
          status_send = STATUS_SEND::BIT_LAST_POS;
          // PORTB.OUTCLR = PIN7_bm;
        }
    default:
        break;

  }
  out_cnt++;
}

volatile uint8_t pressed = 0;
ISR(PORTC_PORT_vect) {
  uint8_t flags = PORTC.INTFLAGS;
  PORTC.INTFLAGS = flags; // clear flags

  uint8_t new_press = 0;

  if (flags & PORT_INT3_bm) new_press = 3;
  else if (flags & PORT_INT4_bm) new_press = 4;
  else if (flags & PORT_INT5_bm) new_press = 5;

  if (new_press != pressed) pressed = new_press;
}

void init_timers() {
  // infrared input PA7: TCB1 in input capture frequency measurement mode
  TCB1.CTRLA = TCB_RUNSTDBY_bm | TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
  TCB1.CTRLB = TCB_CNTMODE_FRQ_gc;
  TCB1.INTCTRL = TCB_CAPT_bm; // enable isr
  TCB1.EVCTRL = TCB_CAPTEI_bm; // enable catpure input event

  // route PA7 (event channel) to TCB1 (event user)
  EVSYS.ASYNCUSER11 = EVSYS_ASYNCUSER11_ASYNCCH0_gc; // set TCB1 event input to channel 0
  EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN7_gc;

  // infrared output PA5: TCB0 37.88kHz 1/4 duty cycle
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc; // | TCB_ENABLE_bm; // max duration: 1/5MHz*2 * 2^16 = 26.2ms
  TCB0.CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc;
  TCB0.CNT = 0;
  TCB0.CCMP = (16<<8) | 66; // LSB: signal period, MSB: duty cycle (LSB+1)*dutycycle/100%

  // 562us steps for infrared output chunks
  TCA0.SINGLE.PER = 2810;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc; // | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
}

void send_cmd(CMD::Type_Cmd cmd) {
  DF("sending: %s",
    cmd == CMD::PWR_ON ? "pwr on" :
    (cmd == CMD::PWR_OFF ? "pwr off" :
    (cmd == CMD::VOL_UP ? "vol up" :
    (cmd == CMD::VOL_DOWN ? "vol down" : "-")))
  );
  cmd_out = pgm_read_dword(&(commands[cmd]));
  DF(" - 0x%08lx\n", cmd_out);
  TCA0.SINGLE.CNT = 0;
  status_send = STATUS_SEND::PREAMBLE_POS;
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

int main(void) {
  uint32_t ts = 0;
  uint8_t button = 0;
  uint8_t keep_awake = 0;
  CMD::Type_Cmd last_cmd = CMD::PWR_OFF;

  // pwm outputs (TCB0: PA5)
  PORTA.DIRSET = PIN5_bm;

  // infrared input PA7 (active high, so we invert it)
  PORTA.DIRCLR = PIN7_bm;
  PORTA.PIN7CTRL |= PORT_INVEN_bm;

  // buttons (active low)
  PORTC.DIR &= ~(PIN5_bm | PIN4_bm | PIN3_bm);
  PORTC.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN4CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN5CTRL = PORT_PULLUPEN_bm;
  _delay_ms(1); // avoid triggering on start
  PORTC.PIN3CTRL |= PORT_ISC_FALLING_gc;
  PORTC.PIN4CTRL |= PORT_ISC_FALLING_gc;
  PORTC.PIN5CTRL |= PORT_ISC_FALLING_gc;

  // oscilloscope test pin
  PORTB.DIRSET = PIN7_bm;

  // deactivate any unused pin (used PA5:ir pwm, PA7: ir input, PB7: debug, PC3-5: buttons)
  PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm; // tosc2
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm; // tosc1
  PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm; // vin
  PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTB.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;

  init_timers();
  mcu_init(0, 0);
  clock.init(511); // 1 rtc tick = 15.6ms

  while (1) {
    // handle buttons (debounced)
    if (pressed && ts == 0) {
      ts = clock.current_tick;
    }
    // we have a valid button press
    else if (ts && (clock.current_tick - ts) > 3) { // 3 ticks * 512 / 32768 = 46.875ms
      if ((PORTC.IN & (1<<pressed)) == 0 && !button) {
        button = pressed;
        keep_awake |= (1<<0);
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
      while (!cmd_sent); // wait to finish sending cmd or repeat
      TCB0.CTRLA &= ~TCB_ENABLE_bm; // just to be sure
      TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
      keep_awake &= ~(1<<0);
      pressed = 0;
      button = 0;
      ts = 0;
    }

    /*
    if (cmd_sent) {
      DL("sent");
      cmd_sent = 0;
    }
    */

    // infrared data detected and ready to use
    if (cmd_received >= 2) {
      DF("rec: 0x%08lx%s\n", cmd_in, cmd_received == 3 ? " (rep)" : "");
      cmd_received = 0;
    }

    // don't go to sleep if sending in progress
    // reception should work while sleeping as all happens in isr
    // active: 9us, sleep: 15.6ms (=512/32768)
    // ir receiver consumption: 340uA
    if (!keep_awake) {
      while (uart_is_busy());
      PORTB.OUTSET = PIN7_bm;
      __asm__ __volatile__ ( "sleep" "\n\t" :: );
      PORTB.OUTCLR = PIN7_bm;
    }
  }
}
