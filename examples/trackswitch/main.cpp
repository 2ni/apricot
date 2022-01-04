#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"
#include "stepper.h"

// https://www.opendcc.de/elektronik/opendecoder/opendecoder_sw_schalt_e.html
// make mcu=attiny1604 flash

// DCC PA5
// LIMIT1 PB1
// LIMIT2 PB0

STEPPER stepper;

namespace DCCPACKET {
  typedef enum {
    WAITPREAMBLE = 0,
    WAITFIRSTDATA = 1,
    WAITNEXTDATA = 2,
    READDATA = 3,
  } Type_State;
}

pins_t INA1 = PA7;
pins_t INA2 = PA6;
pins_t INB1 = PB2;
pins_t INB2 = PB3;
pins_t LIMIT1 = PB1;
pins_t LIMIT2 = PB0;
pins_t DCC = PA5;

volatile int16_t direction = 0;
volatile uint8_t limit_reached = 0;
uint8_t speed = 5;

volatile uint8_t edge_detected = 0;
uint8_t dcc_bit = 0;
DCCPACKET::Type_State state_packet = DCCPACKET::WAITPREAMBLE;

/*
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  switch (in) {
    case 'y':
      direction = -100;
      break;
    case '.':
      direction = 100;
      break;
    case '<':
      direction = -1000;
      break;
    case '-':
      direction = 1000;
      break;
    case '\n':
      pins_flash(&pins_led, 1, 100);
      break;
  }
}
*/

ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // clear flags

  // got DCC edge
  if (flags & PORT_INT5_bm) {
    edge_detected = 1;
  }
}

ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; // clear flags

  // stepper limits
  if (flags & (PORT_INT1_bm | PORT_INT0_bm)) {
    limit_reached = 1;
  }
}

void init_timer() {
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
}


void move_turnout(uint8_t position) {
  if (position == 0x01 && pins_get(&LIMIT2) != 0) {
    pins_set(&pins_led, 1);
    stepper.move(1000, speed);
  }

  if (position == 0x00 && pins_get(&LIMIT1) != 0) {
    pins_set(&pins_led, 1);
    stepper.move(-1000, speed);
  }
}

/*
 * + -> right move
 * - -> left move
 */
int main(void) {
  mcu_init();

  pins_output(&DCC, 0); // set as input
  PORTA.PIN5CTRL |= PORT_ISC_BOTHEDGES_gc; // DCC

  uint8_t preamble_count = 0;
  uint8_t dcc_packets[5] = {0};
  uint8_t dcc_packets_count = 0;
  uint8_t dcc_bit_pos = 0;

  uint8_t wait_for_edge_2 = 0;
  // uint8_t bits[50] = {0};
  // uint8_t bits_pos = 0;

  init_timer();

  // stepper motor
  pins_pullup(&LIMIT1, 1);
  pins_pullup(&LIMIT2, 1);
  PORTB.PIN1CTRL |= PORT_ISC_FALLING_gc; // LIMIT1
  PORTB.PIN0CTRL |= PORT_ISC_FALLING_gc; // LIMIT2

  stepper.init(&INA1, &INA2, &INB1, &INB2);

  // set to left position if unknown at start
  if (pins_get(&LIMIT1) && pins_get(&LIMIT2)) {
    DL("move to home");
    stepper.move(-1000, speed);
    while (pins_get(&LIMIT1)) {
      stepper.loop();
    }
  }

  while (1) {
    // stop stepper on limit
    if (limit_reached) {
      stepper.stop();
      pins_set(&pins_led, 0);
      limit_reached = 0;
    }
    stepper.loop();

    // handle dcc
    if (edge_detected) {
      // 1st edge of the signal
      if (!wait_for_edge_2) {
        TCA0.SINGLE.CNT = 0;
        TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
        wait_for_edge_2 = 1;
      // 2nd edge of the signal
      } else {
        TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
        wait_for_edge_2 = 0;
        dcc_bit = TCA0.SINGLE.CNT < 870; // 10MHz -> 870: 87us. 58us: 1, 116us: 0
        // bits[bits_pos] = dcc_bit;
        // bits_pos++;

        switch (state_packet) {
          case DCCPACKET::WAITPREAMBLE:
            if (dcc_bit) preamble_count++;
            else preamble_count = 0;

            if (preamble_count >= 10) state_packet = DCCPACKET::WAITFIRSTDATA;
            break;
          case DCCPACKET::WAITFIRSTDATA:
            // wait for data after preamble (start bit is 0)
            if (!dcc_bit) {
              state_packet = DCCPACKET::READDATA;
              dcc_bit_pos = 7;
              for (uint8_t i=0; i<dcc_packets_count; i++) dcc_packets[i] = 0;
              dcc_packets_count = 0;
            }
            break;
          case DCCPACKET::WAITNEXTDATA:
            // wait for next packet. If bit is 1 -> end bit
            if (!dcc_bit) {
              state_packet = DCCPACKET::READDATA;
              dcc_bit_pos = 7;
            } else {
              // DF("bits_pos: %u, dcc_packets_count: %u\n", bits_pos, dcc_packets_count);
              // uart_arr("bits", bits, bits_pos);
              // bits_pos = 0;
              uint8_t error = 0;
              for (uint8_t i=0; i<dcc_packets_count; i++) {
                error ^= dcc_packets[i];
              }
              if (!error) {
                // uart_arr("packets", dcc_packets, dcc_packets_count);
                // idle packet
                if (dcc_packets[0] == 0xff) {
                }
                // multifunction decoder
                else if (dcc_packets[0] >= 192 && dcc_packets[0] <= 231) {
                  uint16_t addr = ((dcc_packets[0] & 0x3f) << 8) | dcc_packets[1];
                  uint8_t data_type = dcc_packets[2] & 0xe0;
                  uint8_t data = dcc_packets[2] & 0x1f;
                  DF("addr (14bit): %u (0x%04x) type: 0x%02x, data: 0x%02x\n", addr, addr, data_type, data);
                  if (addr == 2000) {
                    move_turnout(data & 0x01);
                  }
                // basic accessory decoder 9bit:  {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
                } else if (dcc_packets[0] >= 128 && dcc_packets[0] <= 191 && (dcc_packets[1] & 0x80)) {
                  uint16_t addr = (dcc_packets[0] & 0x3f) | ((~dcc_packets[1] & 0x70)<<2);
                  uint8_t action = (dcc_packets[1] & 0x08)>>3;
                  uint8_t local_addr = (dcc_packets[1] & 0x06)>>1;
                  uint8_t output = dcc_packets[1] & 0x01;
                  DF("addr (9bit): %u (0x%04x), local: 0x%02x, output: %u, action: %u\n", addr, addr, local_addr, output, action);
                  if (addr == 267) {
                    move_turnout(output & 0x01);
                  }
                // extended accessory decoder 11bit: {preamble} 0 10AAAAAA 0 0AAA0AA1 0 000XXXXX 0 EEEEEEEE 1
                // of help: https://www.iascaled.com/blog/high-current-dcc-accessory-decoder/
                } else if (dcc_packets[0] >= 128 && dcc_packets[0] <= 191 && (dcc_packets[1] & 0x80) == 0x00) {
                  DL("addr (11bit): not implemented");
                }
              }
              state_packet = DCCPACKET::WAITPREAMBLE;
            }
            break;
          case DCCPACKET::READDATA:
           dcc_packets[dcc_packets_count] |= (dcc_bit<<dcc_bit_pos);
           if (dcc_bit_pos-- == 0) {
             // DF("p: 0x%02x\n", dcc_packets[0]);
             state_packet = DCCPACKET::WAITNEXTDATA;
             dcc_packets_count++;
           }
        }
      }
      edge_detected = 0;
    }
  }
}
