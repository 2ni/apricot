#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
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

namespace ADDR {
  typedef enum {
    NONE,
    BASIC,
    EXTENDED,
  } Type_Addr_Mode;
}

namespace CFG {
  typedef struct {
    uint8_t CV120 = 0x01; // MSB address 261, 0x105
    uint8_t CV121 = 0x05; // LSB address
    uint8_t CV29 = 0x80; // bit 7: 1=accessory decoder, bit 5: 0=extended address mode (alternative: 0xa0)
  } Type_CVs;
}

CFG::Type_CVs cfg_default;
CFG::Type_CVs EEMEM ee_cfg;
CFG::Type_CVs cfg;
uint16_t get_cur_addr() {
  return (uint16_t)(cfg.CV120 << 8 | cfg.CV121);
}

uint16_t last_xor = 0;

pins_t INA1 = PA7;
pins_t INA2 = PA6;
pins_t INB1 = PB2;
pins_t INB2 = PB3;
pins_t LIMIT1 = PB1;
pins_t LIMIT2 = PB0;
pins_t DCC = PA5;

volatile int16_t direction = 0;
volatile uint8_t limit_reached = 0;
uint8_t speed = 2;

volatile uint8_t wait_for_edge_2 = 0;
uint8_t dcc_bit = 0;
volatile DCC_STATE::Type_State state_packet = DCC_STATE::WAITPREAMBLE;
uint8_t in_service_prg = 0;
uint32_t ts_ack_start = 0;
uint32_t ts_last_prg_cmd = 0;
uint32_t ts_stepper = 0;
uint8_t address_tmp = 0;
uint8_t address_last_cv = 0; // 120 if MSB set, 121 if LSB set (according to CV addr)
uint8_t reset_count = 0;

/*
 * trigger for dcc signal
 */
ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // clear flags

  // got DCC edge
  if (flags & PORT_INT5_bm) {
    // 1st edge of the signal
    if (!wait_for_edge_2) {
      TCA0.SINGLE.CNT = 0;
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
      wait_for_edge_2 = 1;
    // 2nd edge of the signal
    } else {
      TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
      wait_for_edge_2 = 2;
    }
  }
}

/*
 * trigger for stepper limits
 */
ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; // clear flags

  // stepper limits
  if (flags & (PORT_INT1_bm | PORT_INT0_bm)) {
    limit_reached = 1;
  }
}

ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
  wait_for_edge_2 = 0;
  state_packet = DCC_STATE::WAITPREAMBLE;
  DL("ovf");
}

void init_timer() {
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
}

void move_turnout(uint8_t position) {
  ts_stepper = clock.current_tick;
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
  // pins_set(&pins_led, 1);
  ts_last_prg_cmd = clock.current_tick;
  // DL("prg on");
}

void deactivate_service_prg() {
  in_service_prg = 0;
  // pins_set(&pins_led, 0);
  ts_last_prg_cmd = 0;
  // DL("prg off");
}

uint8_t is_service_cmd(uint8_t *packets, uint8_t *packets_count) {
  if ((packets[0] & 0xf0) != 0x70) {
    return 0;
  }
  return 1;
}

void cv_bytewise_read(PRG::Type_Prg_Mode mode, uint8_t *cv, uint8_t value) {
  if (mode == PRG::SERVICE && *cv == value) {
    // start consuming >+60mA for 5-7ms if in mode SERVICE
    ts_ack_start = clock.current_tick;
    stepper.keep();
    pins_set(&pins_led, 1);
  }
}

void cv_bytewise_write(PRG::Type_Prg_Mode mode, uint8_t *cv, uint8_t value) {
  // TODO write only after 2nd packet in OPS mode
  *cv = value;
  eeprom_update_block(&cfg, &ee_cfg, sizeof(CFG::Type_CVs));
  // ack in service mode only
  if (mode == PRG::SERVICE) {
    ts_ack_start = clock.current_tick;
    stepper.keep();
    pins_set(&pins_led, 1);
  }
}

void cv_bitwise(PRG::Type_Prg_Mode mode, uint8_t bit_write, uint8_t *cv, uint8_t bit_addr, uint8_t bit_value) {
  // read
  if (mode == PRG::SERVICE && !bit_write && bit_value == ((*cv >> bit_addr) & 0x01)) {
    // start consuming >+60mA for 5-7ms if in mode SERVICE
    ts_ack_start = clock.current_tick;
    stepper.keep();
    pins_set(&pins_led, 1);
  }

  // TODO write only after 2nd packet in OPS mode
  if (bit_write) {
    if (bit_value) *cv |= 1<<bit_addr;
    else *cv &= ~(1<<bit_addr);
    eeprom_update_block(&cfg, &ee_cfg, sizeof(CFG::Type_CVs));
    // ack in service mode only
    if (mode == PRG::SERVICE) {
      // start consuming >+60mA for 5-7ms if in mode SERVICE
      ts_ack_start = clock.current_tick;
      stepper.keep();
      pins_set(&pins_led, 1);
    }
  }
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
 *   bitwise: 111K-DBBB
 *     BBB: bit position (0-7)
 *     K  : 1=write, 0=read
 *     D  : bit value to check/write
 *
 */
void handle_cv(PRG::Type_Prg_Mode mode, uint8_t *packets, uint8_t *packets_count) {
  if ((mode == PRG::SERVICE && !is_service_cmd(packets, packets_count)) || (mode == PRG::OPS && (packets[2] & 0xf0) != 0xe0)) {
    return;
  }

  uint8_t index = mode == PRG::SERVICE ? 0 : 2; // in ops mode first 2 packets contain address (amongst other)
  uint8_t cmd_type = (packets[index] & 0x0c) >> 2;
  uint16_t cv = (((packets[index] & 0x03) << 8) | packets[index+1]) + 1;
  uint8_t bit_write = 0;
  uint8_t bit_addr = 0;
  uint8_t value = 0;

  if (cmd_type == 0x02) {
    bit_write = packets[index+2] & 0x10;
    value = (packets[index+2] & 0x08) >> 3;
    bit_addr = packets[index+2] & 0x07;
    // DF("cv#%u: %s bit%u %u\n", cv, bit_write ? "write" : "read", bit_addr, value);
  } else {
    value = packets[index+2];
    // DF("cv#%u: %s 0x%02x\n", cv, cmd_type == 0x03 ? "write" : "read", packets[index+2]);
  }

  // cv#120: MSB, cv#121: LSB
  switch (cv) {
    case 120:
    case 121:
      switch (cmd_type) {
        case 0x01: // read/verify
          cv_bytewise_read(mode, cv == 121 ? &cfg.CV121 : &cfg.CV120, value);
          break;
        // write
        case 0x03:
          // start consuming >+60mA for 5-7ms
          // in service mode we can directly set the msb or lsb as no address is used to communicate
          if (mode == PRG::SERVICE) {
            cv_bytewise_write(mode, cv == 120 ? &cfg.CV120 : &cfg.CV121, value);
            // DF("new address: %u/0x%03x\n", get_cur_addr(), get_cur_addr());
          }

          if (mode == PRG::OPS && ((address_last_cv == 121 && cv == 120) || (address_last_cv == 120 && cv == 121))) {
            cfg.CV121 = cv == 121 ? value : address_tmp;
            cfg.CV120 = cv == 120 ? value : address_tmp;
            eeprom_update_block(&cfg, &ee_cfg, sizeof(CFG::Type_CVs));
            address_tmp = 0;
            address_last_cv = 0;
            // DF("new address: %u/0x%03x\n", get_cur_addr(), get_cur_addr());
          } else {
            last_xor = 0;
            address_tmp = value;
            address_last_cv = cv;
          }
          break;
        case 0x02:
          break;
      }
      break;
    case 29:
      switch (cmd_type) {
        case 0x01: // read/verify
          cv_bytewise_read(mode, &cfg.CV29, value);
          break;
        case 0x03: // write
          cv_bytewise_write(mode, &cfg.CV29, value);
          break;
        case 0x02: // bitwise
          cv_bitwise(mode, bit_write, &cfg.CV29, bit_addr, value);
          break;
      }
      break;
  }
}

/*
 * use macro as function seems to be too slow in main loop
 * a function leads ot packet errors
 */
#define STEPPER_REACHED_POS(break_when_reached) { \
  if (limit_reached || (ts_stepper && (clock.current_tick - ts_stepper) > 4096)) { \
    ts_stepper = 0; \
    stepper.stop(); \
    stepper.move(0, speed); \
    pins_set(&pins_led, 0); \
    limit_reached = 0; \
    if (break_when_reached) break; \
  } \
}


/*
 * + -> right move
 * - -> left move
 */
int main(void) {
  mcu_init();

  eeprom_read_block(&cfg, &ee_cfg, sizeof(CFG::Type_CVs));
  if ((cfg.CV120 & cfg.CV121 & cfg.CV29) == 0xff) cfg = cfg_default;

  DF("CV120: 0x%02x\n", cfg.CV120);
  DF("CV121: 0x%02x\n", cfg.CV121);
  DF("CV29: 0x%02x\n", cfg.CV29);
  DF("current address: %u (0x%02x)\n", get_cur_addr(), get_cur_addr());

  pins_output(&DCC, 0); // set as input
  PORTA.PIN5CTRL |= PORT_ISC_BOTHEDGES_gc; // DCC

  // dbg for oscilloscope
  // pins_output(&PA3, 1);
  // pins_set(&PA3, 1);

  uint8_t preamble_count = 0;
  uint8_t dcc_packets[6] = {0};
  uint8_t dcc_packets_count = 0;
  uint8_t dcc_bit_pos = 0;

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
    ts_stepper = clock.current_tick;
    DL("move to home");
    stepper.move(-1000, speed);
    ts_stepper = clock.current_tick;
    while (1) {
      STEPPER_REACHED_POS(1);
      stepper.loop();
    }
  }

  DL("waiting for signal...");
  while (1) {
    // stop stepper on limit or max duration for stepper reached (1s/(8/32768))
    stepper.loop();
    STEPPER_REACHED_POS(0);

    // handle dcc
    // TODO?
    // if a eeprom_update is needed, it takes too much time and bits of the next packet get lost
    // usually this is a idle packet and therefore no problem
    // possible solution: use a ring buffer for incoming bits and process them
    // could potentially lead to other issues, eg ack too late, ...
    if (wait_for_edge_2 == 2) {
      wait_for_edge_2 = 0;
      if (TCA0.SINGLE.CNT > 1450) {
        DF("* %u\n", TCA0.SINGLE.CNT);
        state_packet = DCC_STATE::WAITPREAMBLE;
        preamble_count = 0;
        continue;
      }

      dcc_bit = TCA0.SINGLE.CNT < 870; // 10MHz -> 870: 87us. 58us: 1, 116us: 0
      // bits[bits_pos++] = dcc_bit;

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
            uint8_t error = 0;
            for (uint8_t i=0; i<dcc_packets_count; i++) {
              error ^= dcc_packets[i];
            }

            // DF("bits_pos: %u, dcc_packets_count: %u\n", bits_pos, dcc_packets_count);
            // uart_arr("bits", bits, bits_pos);
            // bits_pos = 0;

            /**************************************************************************
             * packet handling starts here
             **************************************************************************/
            if (error) {
              uart_arr("err", dcc_packets, dcc_packets_count);
            }
            if (!error) {
              ADDR::Type_Addr_Mode addr_mode = ADDR::NONE;
              if (dcc_packets[0] >= 128 && dcc_packets[0] <= 191 && (dcc_packets[1] & 0x80)) addr_mode = ADDR::BASIC;
              else if (dcc_packets[0] >= 128 && dcc_packets[0] <= 191 && (dcc_packets[1] & 0x89) == 0x01) addr_mode = ADDR::EXTENDED;
              // extended: {preamble} 0 10AAAAAA 0 0AAA0AA1 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
              // basic:    {preamble} 0 10AAAAAA 0 1AAA1DD0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
              uint8_t is_prg_ops_cmd = dcc_packets_count == 6 && (dcc_packets[2] & 0xf0) == 0xe0 && (
                  (addr_mode == ADDR::BASIC && (dcc_packets[1] & 0x89) == 0x88) ||
                  (addr_mode == ADDR::EXTENDED && (dcc_packets[1] & 0x89) == 0x01)
                  );
              uint8_t is_reset_packet = dcc_packets_count == 3 && dcc_packets[0] == 0x00 && dcc_packets[1] == 0x00;

              // reset reset_count if not reached service mode
              if (reset_count && !is_reset_packet) reset_count = 0;

              // reset check for 2nd prg cmd in ops mode if not same
              if (last_xor && !is_prg_ops_cmd) last_xor = 0;

              // reset temp address if no prg cmd
              if (address_last_cv && !is_prg_ops_cmd) address_last_cv = 0;

              // TODO and if not reset packet
              /*
              if (in_service_prg && !is_service_cmd(dcc_packets, &dcc_packets_count)) {
                deactivate_service_prg();
              }
              */

              // sniffing all packets except idle / reset
              // idle:  ff 00 ff, reset: 00 00 00
              /*
              if (!(dcc_packets_count == 3 && (
                  (dcc_packets[0] == 0xff && dcc_packets[1] == 0x00 && dcc_packets[2] == 0xff)
                  || (dcc_packets[0] == 0x00 && dcc_packets[1] == 0x00 && dcc_packets[2] == 0x00)
               ))) {
                uart_arr("", dcc_packets, dcc_packets_count);
              }
              */

              // reset packet
              // go into service mode if >= 25 reset packets received, stay as long as reset packets sent
              if (is_reset_packet) {
                if (!in_service_prg) {
                  reset_count++;
                  // DF("%u\n", reset_count);
                  if (reset_count >= 25) {
                    activate_service_prg();
                  }
                } else if (ts_last_prg_cmd) {
                  ts_last_prg_cmd = clock.current_tick;
                }
              }
              else if (in_service_prg) {
                handle_cv(PRG::SERVICE, dcc_packets, &dcc_packets_count);
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
              else if (addr_mode == ADDR::BASIC) {
                // basic accessory 11bit MADA (https://wiki.rocrail.net/doku.php?id=addressing:accessory-pg-de)
                // uint8_t activation = (dcc_packets[1] & 0x08)>>3; // "C" ("activated/deactivated")
                uint8_t port = ((dcc_packets[1] & 0x06)>>1) + 1; // "DD"
                uint16_t module_addr = (dcc_packets[0] & 0x3f) | ((uint16_t)((~dcc_packets[1] & 0x70))<<2);
                uint16_t addr = (module_addr - 1) * 4 + port;
                uint8_t output = dcc_packets[1] & 0x01; // "R" 0=left, 1=right ("which coil should be de/activated")
                // DF("basic: %u (0x%04x), activation: %u, output: %u\n", addr, addr, activation, output);
                if (!(cfg.CV29 & (1<<5)) && addr == get_cur_addr()) {
                  // cv access (ops mode): 10AAAAAA 0 1AAA1DD0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
                  if (is_prg_ops_cmd) {
                    if (!last_xor) last_xor = dcc_packets[5];
                    else if (last_xor == dcc_packets[5]) {
                      handle_cv(PRG::OPS, dcc_packets, &dcc_packets_count);
                    }
                  } else {
                    move_turnout(output & 0x01);
                  }
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
              else if (addr_mode == ADDR::EXTENDED) {
                uint16_t addr = ((dcc_packets[0] & 0x3f)<<2) | ((uint16_t)((~dcc_packets[1] & 0x70))<<4) | ((dcc_packets[1] & 0x06)>>1);
                // DF("extended: (11bit): %u (0x%04x), data: 0x%02x\n", addr, addr, dcc_packets[2]);
                if ((cfg.CV29 & (1<<5)) && addr == get_cur_addr()) {
                  if (is_prg_ops_cmd) {
                    if (!last_xor) last_xor = dcc_packets[5];
                    else if (last_xor == dcc_packets[5]) {
                      handle_cv(PRG::OPS, dcc_packets, &dcc_packets_count);
                    }
                  } else {
                    move_turnout(dcc_packets[2] & 0x01);
                  }
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

    // ack signal
    if (ts_ack_start && (clock.current_tick - ts_ack_start) > 24) { // 0.006s/(8/32768)
      ts_ack_start = 0;
      // stop consuming
      stepper.stop();
      pins_set(&pins_led, 0);
    }

    // timeout prg mode
    if (ts_last_prg_cmd && (clock.current_tick - ts_last_prg_cmd) > 122) { // 0.030s/(8/32768)
      deactivate_service_prg();
    }
  }
}
