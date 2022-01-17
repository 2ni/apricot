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

namespace DCC_STATE {
  typedef enum {
    WAITPREAMBLE = 0,
    WAITFIRSTDATA = 1,
    WAITNEXTDATA = 2,
    READDATA = 3,
  } Type_State;
}

namespace PRG {
  typedef enum {
    SERVICE = 0,
    OPS = 1,
  } Type_Prg_Mode;
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
DCC_STATE::Type_State state_packet = DCC_STATE::WAITPREAMBLE;
uint8_t in_service_prg = 0;
uint32_t ts_ack_start = 0;
uint32_t ts_last_prg_cmd = 0;
uint16_t address = 267;
uint16_t address_tmp = 0;
uint8_t address_in_update = 0; // 0x01 if LSB set, 0x02 if MSB set
uint8_t reset_count = 0;

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

void activate_service_prg() {
  reset_count = 0;
  in_service_prg = 1;
  pins_set(&pins_led, 1);
  ts_last_prg_cmd = clock.current_tick;
  DL("prg on");
}

void deactivate_service_prg() {
  in_service_prg = 0;
  pins_set(&pins_led, 0);
  ts_last_prg_cmd = 0;
  address_tmp = 0;
  address_in_update = 0;
  DL("prg off");
}

uint8_t is_service_cmd(uint8_t *packets, uint8_t *packets_count) {
  if ((packets[0] & 0xf0) != 0x70) {
    return 0;
  }
  return 1;
}

/*
 *
 * PRG::SERVICE cv access  {preamble} 0                       0111CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 * PRG::OPS     cv access  {preamble} 0 10AAAAAA 0 1AAA1DD0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *   CC=00: reserved
 *   CC=01: verify byte
 *   CC=11: write byte
 *   CC=01: bit manipulation
 *   VVVVVVVVVV: 10bit CV address
 *   DDDDDDDD  : CV value
 *
 * address_tmp is only cleared if prg mode is left
 */
void update_cv(PRG::Type_Prg_Mode mode, uint8_t *packets, uint8_t *packets_count) {
  if ((mode == PRG::SERVICE && !is_service_cmd(packets, packets_count)) || (mode == PRG::OPS && (packets[2] & 0xf0) != 0xe0)) {
    return;
  }

  if (in_service_prg) {
    ts_last_prg_cmd = clock.current_tick;
  }

  uint8_t index = mode == PRG::SERVICE ? 0 : 2;
  uint8_t cmd_type = (packets[index] & 0x0c) >> 2;
  uint16_t cv = (((packets[index] & 0x03) << 8) | packets[index + 1]) + 1;
  DF("cv#%u: %u\n", cv, packets[index + 2]);
  if (cv == 1 || cv == 9) {
    switch (cmd_type) {
      // verify
      case 0x01:
        if (packets[index + 2] == (cv == 1 ? address & 0x00ff : (address & 0xff00) >> 8)) {
          // start consuming >+60mA for 5-7ms if in mode SERVICE
          if (mode == PRG::SERVICE) {
            ts_ack_start = clock.current_tick;
            stepper.keep();
            DL("ack start");
          }
        }
        break;
      // write
      case 0x03:
        // start consuming >+60mA for 5-7ms
        if (mode == PRG::SERVICE) {
          ts_ack_start = clock.current_tick;
          stepper.keep();
          DL("ack start");
        }

        if ((address_in_update == 0x01 && cv == 9) || (address_in_update == 0x02 && cv == 1)) {
          address = address_tmp | (packets[index + 2] << (cv == 1 ? 0 : 8));
          address_tmp = 0;
          DF("new address: 0x%04x\n", address);
        } else {
          address_tmp = packets[index + 2] << (cv == 1 ? 0 : 8);
          address_in_update |= cv == 1 ? 0x01 : 0x02;
        }
        break;
    }
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
  uint8_t dcc_packets[6] = {0};
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
          case DCC_STATE::WAITPREAMBLE:
            if (dcc_bit) preamble_count++;
            else preamble_count = 0;

            if (preamble_count >= 10) state_packet = DCC_STATE::WAITFIRSTDATA;
            break;
          case DCC_STATE::WAITFIRSTDATA:
            // wait for data after preamble (start bit is 0)
            if (!dcc_bit) {
              state_packet = DCC_STATE::READDATA;
              dcc_bit_pos = 7;
              for (uint8_t i=0; i<dcc_packets_count; i++) dcc_packets[i] = 0;
              dcc_packets_count = 0;
            }
            break;
          case DCC_STATE::WAITNEXTDATA:
            // get packets. If bit is 1 -> end bit
            if (!dcc_bit) {
              state_packet = DCC_STATE::READDATA;
              dcc_bit_pos = 7;
            } else {
              // DF("bits_pos: %u, dcc_packets_count: %u\n", bits_pos, dcc_packets_count);
              // uart_arr("bits", bits, bits_pos);
              // bits_pos = 0;
              uint8_t error = 0;
              for (uint8_t i=0; i<dcc_packets_count; i++) {
                error ^= dcc_packets[i];
              }

              /**************************************************************************
               * packet handling starts here
               **************************************************************************/
              if (!error) {
                // TODO and if not reset packet
                if (in_service_prg && !is_service_cmd(dcc_packets, &dcc_packets_count)) {
                  // deactivate_service_prg();
                }

                /*
                if (dcc_packets[0] != 0xff) {
                  uart_arr("detected", dcc_packets, dcc_packets_count);
                }
                */

                if (in_service_prg) {
                  update_cv(PRG::SERVICE, dcc_packets, &dcc_packets_count);
                }

                // idle packet
                else if (dcc_packets[0] == 0xff) {
                }
                // reset packet
                else if (dcc_packets[0] == 0x00 && dcc_packets[1] == 0x00) {
                  if (!in_service_prg) {
                    reset_count++;
                    if (reset_count >= 25) {
                      activate_service_prg();
                    }
                  } else if (ts_last_prg_cmd) {
                    ts_last_prg_cmd = clock.current_tick;
                  }
                }
                // multifunction decoder 14bit
                // {preamble} 0 11AAAAAA 0 AAAAAAAA 0 CCCDDDDD 0 EEEEEEEE 1
                // cv access short form only in ops mode (for 7bit or 14bt) for engines (see RCN214, p.6)
                // basic accessory decoder 9bit   {preamble} 0 10AAAAAA 0 1AAACDDR 0 EEEEEEEE 1
                // basic accessory decoder 11bit  {preamble} 0 10AAAAAA 0 1AAACAAR 0 EEEEEEEE 1
                //
                // broadcast command              {preamble} 0 10111111 0 1000CDDR 0 EEEEEEEE 1
                // emergency stop                 {preamble} 0 10111111 0 10000110 0 EEEEEEEE 1
                // reset (addr 0, data 0)         {preamble} 0 00000000 0 00000000 0 00000000 1
                // RN214:
                // cv access (ops mode)           {preamble} 0 10AAAAAA 0 1AAA1DD0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1

                // RCN216/RCN211: service mode: 25 reset packet, then prg packet starting with 0111 (at least 2 similar)
                // RCN216: service mode ack: 5-7ms +60mA power consumption after reception of every packet
                // RCN216: leave prg modus if normal packet or 30ms since reset/cv access packet
                // beim Programmiermodus mit isoliertem Gleisabschnitt: entsprechenden Pakete beginnen direkt mit den Befehlsbytes ohne Adresse
                else if (dcc_packets[0] >= 128 && dcc_packets[0] <= 191 && (dcc_packets[1] & 0x80)) {
                  // cv access (ops mode): 10AAAAAA 0 1AAA1DD0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD
                  if (dcc_packets_count > 3 && (dcc_packets[1] & 0x89) == 0x88 && (dcc_packets[2] & 0xf0) == 0xe0) {
                    update_cv(PRG::OPS, dcc_packets, &dcc_packets_count);
                  }
                  else {
                    // basic accessory 11bit MADA (https://wiki.rocrail.net/doku.php?id=addressing:accessory-pg-de)
                    uint8_t activation = (dcc_packets[1] & 0x08)>>3; // "C" ("activated/deactivated")
                    uint8_t port = ((dcc_packets[1] & 0x06)>>1) + 1; // "DD"
                    uint16_t module_addr = (dcc_packets[0] & 0x3f) | ((uint16_t)((~dcc_packets[1] & 0x70))<<2);
                    uint16_t addr = (module_addr - 1) * 4 + port;
                    uint8_t output = dcc_packets[1] & 0x01; // "R" 0=left, 1=right ("which coil should be de/activated")
                    DF("basic: %u (0x%04x), activation: %u, output: %u\n", addr, addr, activation, output);
                    if (addr == address) {
                      move_turnout(output & 0x01);
                    }

                    // 11bit (output=0 -> left, output=1 -> right)
                    /*
                    addr = ((dcc_packets[0] & 0x3f)<<2) | ((uint16_t)((~dcc_packets[1] & 0x70))<<4) | ((dcc_packets[1] & 0x06)>>1);
                    activation = (dcc_packets[1] & 0x08)>>3;
                    output = dcc_packets[1] & 0x01;
                    DF("basic (11bit): %u (0x%04x), activation: %u, output: %u\n", addr, addr, activation, output);
                    */
                  }
                // extended accessory decoder 11bit  {preamble} 0 10AAAAAA 0 0AAA0AA1 0 000XXXXX 0 EEEEEEEE 1
                // broadcast command                 {preamble} 0 10111111 0 00000111 0 000XXXXX 0 EEEEEEEE 1
                // emergency stop                    {preamble} 0 10111111 0 00000111 0 00000000 0 EEEEEEEE 1
                // cv access (ops mode)              {preamble} 0 10AAAAAA 0 0AAA0AA1 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
                // of help: https://www.iascaled.com/blog/high-current-dcc-accessory-decoder/
                //          https://dccwiki.com/Configuration_Variable
                } else if (dcc_packets[0] >= 128 && dcc_packets[0] <= 191 && (dcc_packets[1] & 0x89) == 0x01) {
                  uint16_t addr = ((dcc_packets[0] & 0x3f)<<2) | ((uint16_t)((~dcc_packets[1] & 0x70))<<4) | ((dcc_packets[1] & 0x06)>>1);
                  DF("extended: (11bit): %u (0x%04x), data: 0x%02x\n", addr, addr, dcc_packets[2]);
                  if (addr == address) {
                    move_turnout(dcc_packets[2] & 0x01);
                  }
                }
              }
              state_packet = DCC_STATE::WAITPREAMBLE;
            }
            break;
          case DCC_STATE::READDATA:
           dcc_packets[dcc_packets_count] |= (dcc_bit<<dcc_bit_pos);
           if (dcc_bit_pos-- == 0) {
             // DF("p: 0x%02x\n", dcc_packets[0]);
             state_packet = DCC_STATE::WAITNEXTDATA;
             dcc_packets_count++;
           }
        }
      }
      edge_detected = 0;
    }
    // create ack signal
    if (ts_ack_start && (clock.current_tick - ts_ack_start) > 24) { // 0.006s/(8/32768)
      ts_ack_start = 0;
      // stop consuming
      stepper.stop();
      DL("ack stop");
    }

    // timeout prg mode
    if (ts_last_prg_cmd && (clock.current_tick - ts_last_prg_cmd) > 122) { // 0.030s/(8/32768)
      deactivate_service_prg();
    }
  }
}
