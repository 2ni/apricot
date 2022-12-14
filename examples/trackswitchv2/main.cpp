#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "dcc_structs.h"
#include "queue.h"

/*
 * define prototypes to avoid "not declared in scope"
 */
void parse_accessory_opmode(DCC::PACKET packet);
void process_extended(uint16_t addr, uint8_t data);
void process_basic_output(uint16_t address, uint8_t power, uint8_t direction);
void process_basic_decoder(uint16_t address, uint8_t port, uint8_t power, uint8_t direction);
void parse_cv(DCC::MODE mode, uint8_t cmd, uint16_t cv_addr, uint8_t cv_data);
uint8_t read_cv(uint16_t cv_addr);
uint8_t write_cv(uint16_t cv_addr, uint8_t cv_data);
void ack_cv(DCC::MODE mode);
uint8_t get_memory_index(uint16_t cv_addr);
void factory_default();
void load_config();


/*
 * make mcu=attiny1604 flash
 * DCC PA5
 * LEDR PA3 (position = 0)
 * LEDG PA4 (position = 1)
 *
 * see https://pastebin.com/tEREeBg9 for an example of dcc with ATtiny412-SSN
 */

static const uint8_t INVALID_BIT = -1;

volatile DCC::STATE state = DCC::STATE_PREAMBLE;
DCC::PACKET packet;
uint8_t xor_value = 0;
uint8_t bit_count = 0;
uint8_t byte = 0;
QUEUE queue;

uint8_t EEMEM ee_cfg[DCC::CV_SIZE];
DCC::CFG cfg;

void reset() {
  state = DCC::STATE_PREAMBLE;
  packet.len = 0;
  packet.preamble = 0;
  byte = 0;
  xor_value = 0;
  bit_count = 0;
}

void process_bit(uint8_t bit) {
  switch (state) {
    case DCC::STATE_PREAMBLE:
      if (bit) {
        packet.preamble++;
        if (packet.preamble >= 12) {
          state = DCC::STATE_START_BIT;
        }
      } else {
        packet.preamble = 0;
      }
      break;
    // wait for start bit (0)
    case DCC::STATE_START_BIT:
      if (bit) {
        packet.preamble++;
      } else {
        packet.len = 0;
        xor_value = 0;

        state = DCC::STATE_DATA_BYTE;
      }
      break;
    case DCC::STATE_DATA_BYTE:
      byte = (byte<<1) | bit;
      if (++bit_count == 8) {
        state = DCC::STATE_END_BIT;
        if (packet.len < 6) {
          packet.data[packet.len] = byte;
        }
        packet.len++;
        xor_value ^= byte;
        bit_count = 0;
      }
      break;
    case DCC::STATE_END_BIT:
      if (bit) {
        if (xor_value == 0 && (3 <= packet.len) && (packet.len < 6)) {
          // valid
          queue.push(packet);
        }
        state = DCC::STATE_PREAMBLE;
        packet.preamble = 0;
      } else {
        state = DCC::STATE_DATA_BYTE;
      }
      break;
  }
}

void parse_packet(DCC::PACKET packet) {
  uint8_t addr = packet.data[0];

  if ((0 <= addr) && (addr <= 127)) {
    DF("multifunction decoder 7bit, not implemented, pre: %u, ", packet.preamble);
    uart_arr("packet", packet.data, packet.len);
  } else if ((128 <= addr) && (addr <= 191)) {
    DF("basic/extended accessory 9/11bit, pre: %u, ", packet.preamble);
    uart_arr("packet", packet.data, packet.len);
    parse_accessory_opmode(packet);
  } else if ((192 <= addr) && (addr <= 231)) {
    DF("multifunction devoder 14bit, not implemented, pre: %u, ", packet.preamble);
    uart_arr("packet", packet.data, packet.len);
  } else if ((232 <= addr) && (addr <= 254)) {
    DF("future use, not implemented, pre: %u, ", packet.preamble);
    uart_arr("packet", packet.data, packet.len);
  } else if (addr == 255 && packet.data[1] == 0 && packet.len == 3) {
    // idle packet
  };
}

/*
 *  basic (decoder addr)  9bit: {preamble} 0 10AAAAAA 0 1aaaCDDR 0 EEEEEEEE 1
 *  basic (output addr)  11bit: {preamble} 0 10AAAAAA 0 1aaaCAAR 0 EEEEEEEE 1
 *  extended 11bit:             {preamble} 0 10AAAAAA 0 0aaa0AA1 0 DDDDDDDD 0 EEEEEEEE 1
 *
 *  A: address bit
 *  a: address bit complement
 *  C: power (0=inactive, 1=active)
 *  D: port (0-3)
 *  R: direction (0:left/diverting/stop, 1:right/straight/drive)
 *
 * http://normen.railcommunity.de/RCN-213.pdf
 * https://dccwiki.com/Accessory_Decoder_Addressing
 * Decoder 1, Port 1, 2, 3, 4
 * Decoder 2, Port 5, 6, 7, 8
 * ...
 * Addr 5: (2-1)*4 + 0 + 1
 * -> address (decoder port address: (decoder-1)*4 + port[0-3] + 1
 */
void parse_accessory_opmode(DCC::PACKET packet) {
  uint8_t p1 = packet.data[1];
  uint8_t power = (p1 & 0x08)>>3; // "C" 0=inactive, 1=active
  uint8_t port = ((p1 & 0x06)>>1); // "DD"
  uint8_t direction = p1 & 0x01; // "R" 0=left/diverting/stop, 1=right/straight/drive
  uint16_t addr_decoder = (packet.data[0] & 0x3f) | ((uint16_t)((~p1 & 0x70))<<2); // 9bit, CV29[6]=0
  uint16_t addr_output = ((((addr_decoder - 1) << 2) | port) + 1); // 11bit, CV29[6]=1
  DF("   9bit: %04u, power: %u, port: %u, dir: %u\n", addr_decoder, power, port, direction);
  DF("  11bit: %04u, power: %u, port: -, dir: %u\n", addr_output, power, direction);

  // check address
  if (((cfg.cv29 & 0x40) && addr_output != cfg.addr) || (!(cfg.cv29 & 0x40) && addr_decoder != cfg.addr)) return;

  if (packet.len == 3 && (p1 & 0x80) == 0x80) {
    // basic
    if ((cfg.cv29 & 0x40)) {
      process_basic_output(addr_output, power, direction);
    } else {
      process_basic_decoder(addr_decoder, port, power, direction);
    }
  } else if (packet.len == 4 && (p1 & 0x89) == 0x01) {
    // extended
    process_extended(addr_output, packet.data[2]);
  } else if (packet.len == 6 && ((p1 & 0x80) == 0x80 || (p1 & 0x89) == 0x89) && (packet.data[2] & 0xf0) == 0xe0) {
    // ops mode configuration
    uint8_t cmd = (packet.data[2] & 0x0c) >> 2;
    uint16_t cv_addr = (((packet.data[2] & 0x03) << 8) | packet.data[3]) + 1;
    parse_cv(DCC::OPS, cmd, cv_addr, packet.data[4]);
  }
}

void process_basic_output(uint16_t address, uint8_t power, uint8_t direction) {
}

void process_basic_decoder(uint16_t address, uint8_t port, uint8_t power, uint8_t direction) {
}

void process_extended(uint16_t address, uint8_t data) {
}

/*
 * ack only in service mode
 * directly set msb/lsb addr in service mode (in ops mode we would loose contact, as addr would temporary change)
 *
 *  ops mode basic:    {preamble} 0 10AAAAAA 0 1aaaCDDR 0 1110KKVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *  ops mode extended: {preamble} 0 10AAAAAA 0 1aaa0AA1 0 1110KKVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *  service mode:      {preamble}                       0 0111KKVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *
 *  ops mode (bit):    {preamble}                       0 110110VV 0 VVVVVVVV 0 111KDBBB 0 EEEEEEEE 1
 *  service mode (bit):{preamble}                       0 011110VV 0 VVVVVVVV 0 111KDBBB 0 EEEEEEEE 1
 *
 *  A: address bit
 *  a: address bit complement
 *  C: power (0=inactive, 1=active)
 *  D: port (0-3)
 *  R: direction (0:left/diverting/stop, 1:right/straight/drive)
 *  K: edit (01=verify, 11=write, 10=bit manipulation), bit: (1=write, 0=verify)
 *  V: CV address
 *  B: bit address
*/
void parse_cv(DCC::MODE mode, uint8_t cmd, uint16_t cv_addr, uint8_t cv_data) {
  switch (cmd) {
    case 0x01: // read/verify
      if (read_cv(cv_addr) == cv_data) {
        ack_cv(mode);
      }
      break;
    case 0x03: // write
      if (write_cv(cv_addr, cv_data)) {
        ack_cv(mode);
      }
      break;
    case 0x02: // bitwise read/write
      if ((cv_data & 0xe0) != 0xe0) return;
      uint8_t bit_cmd = (cv_data & 0x10) >> 4;
      uint8_t bit_mask = 1<<(cv_data & 0x07);
      uint8_t bit = ((cv_data & 0x08) >> 3) * bit_mask;
      switch (bit_cmd) {
        case 1: // write
          cv_data = (read_cv(cv_addr) & ~bit_mask) | bit; // overwrite with new bit
          if ((write_cv(cv_addr, cv_data) & bit_mask) == bit) {
            ack_cv(mode);
          }
          break;
        case 0: // verify
          if ((read_cv(cv_addr) & bit_mask) == bit) {
            ack_cv(mode);
          }
          break;
      }
      break;
  }
}

uint8_t read_cv(uint16_t cv_addr) {
  uint8_t index = get_memory_index(cv_addr);
  if (index == 0xff) return index;

  return eeprom_read_byte(&ee_cfg[index]);
}

uint8_t write_cv(uint16_t cv_addr, uint8_t cv_data) {
  uint8_t update_eeprom = 0;
  uint8_t update_config = 0;

  switch (cv_addr) {
    case 8: // manufacturer id
      factory_default();
      update_config = 1;
      break;
    case 1: // addr (lsb, msb)
    case 9:
    case 29: // config
      update_eeprom = 1;
      update_config = 1;
    default: // eg version
      break;
  }

  if (update_eeprom) {
    eeprom_update_byte(&ee_cfg[get_memory_index(cv_addr)], cv_data);
  }

  if (update_config) {
    load_config();
  }
  return read_cv(cv_addr);
}

void ack_cv(DCC::MODE mode) {
}

/*
 * get position of CV in eeprom
 */
uint8_t get_memory_index(uint16_t cv_addr) {
  uint8_t index = 0;
  switch (cv_addr) {
    case 1:
      index = DCC::CV01_ADDR_LSB;
      break;
    case 9:
      index = DCC::CV09_ADDR_MSB;
      break;
    case 7:
      index = DCC::CV07_VERSION;
      break;
    case 8:
      index = DCC::CV08_MANUFACTURER;
      break;
    case 29:
      index = DCC::CV29_CONFIG;
      break;
    default:
      return 0xff;
      break;
  }
  return index;
}

void factory_default() {
  DL("factory reset");
  for (uint8_t i=0; i<(sizeof(DCC::cv_defaults)/sizeof(DCC::CV)); i++) {
    eeprom_update_byte(&ee_cfg[DCC::cv_defaults[i].index], DCC::cv_defaults[i].data);
  }
}

void load_config() {
  cfg.addr = (uint16_t)(read_cv(9) << 8 | read_cv(1));
  cfg.cv29 = read_cv(29);

  DL("loaded into config:");
  DF("CV29: 0x%02x\n", cfg.cv29);
  DF("current address: %u (0x%04x)\n", cfg.addr, cfg.addr);
}

ISR(TCB0_INT_vect) {
  uint16_t width = TCB0.CCMP; // 0.1us (500 = 50us)

  // 1: 52-64us ±6
  // 0: 116us
  if ((460 <= width) && (width < 700)) process_bit(1);
  else if ((900 <= width) && (width < 10000)) process_bit(0);
  else reset();
}

int main(void) {
  mcu_init();

  // load CVs
  if (read_cv(8) != 0x0d) {
    factory_default();
  }
  load_config();
  write_cv(7, 2);

  EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc; // ASYNCUSER0 = TCB0, connect with channel 0
  EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN5_gc; // DCC PA5

  TCB0.CTRLB = TCB_CNTMODE_PW_gc; // input capture pulse width
  TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm | TCB_FILTER_bm; // edge: signal inverted
  TCB0.INTCTRL = TCB_CAPT_bm; // enable capture interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  DL("waiting for data...");

  DCC::PACKET p;
  while (1) {
    queue.pull(p);
    if (p.len) {
      parse_packet(p);
    }
  }
}
