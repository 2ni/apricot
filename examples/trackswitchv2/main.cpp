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
void process_extended(uint8_t data);
void process_basic_output(uint8_t power, uint8_t direction);
void process_basic_decoder(uint8_t port, uint8_t power, uint8_t direction);
void parse_servicemode(DCC::PACKET packet);
void parse_cv(DCC::MODE mode, uint8_t cmd, uint16_t cv_addr, uint8_t cv_data);
uint8_t read_cv(uint16_t cv_addr);
uint8_t write_cv(uint16_t cv_addr, uint8_t cv_data, uint8_t do_not_load_config = 0);
void ack_cv(DCC::MODE mode);
uint8_t get_memory_index(uint16_t cv_addr);
void factory_default();
void load_config();
void toggle_track(uint8_t direction);
void toggle_learning_mode(uint8_t force_value = 0);
void position_reached();


/*
 * make mcu=attiny1604 flash
 * PA5: DCC
 * PA3: LEDR (position = 0, left/diverting/stop)
 * PA4: LEDG (position = 1, right/straight/run))
 * PB2/PB3: MOTOR
 * PA7: TOGGLE LEARN MODE
 * PB0: sensor limit pos 0 (pullup, active 0)
 * PB1: sensor limit pos 1 (pullup, active 0)
 *
 * see https://pastebin.com/tEREeBg9 for an example of dcc with ATtiny412-SSN
 */

// #define __LIMITSENSORS_ENABLED__

#define PORT_LED PORTA
#define LED_RED PIN3_bm
#define LED_GREEN PIN4_bm
#define PORT_MOTOR PORTB
#define MOTORIN1 PIN2_bm
#define MOTORIN2 PIN3_bm

static const uint8_t INVALID_BIT = -1;

volatile DCC::STATE state = DCC::STATE_PREAMBLE;
DCC::PACKET packet;
uint8_t xor_value = 0;
uint8_t bit_count = 0;
uint8_t byte = 0;
QUEUE queue;
uint8_t reset_packet_count = 0;

uint8_t EEMEM ee_cfg[DCC::CV_SIZE];
DCC::CFG cfg;

uint32_t ts = 0;
volatile uint32_t ts_debounce = 0;
volatile uint8_t is_learning = 0;

void reset() {
  state = DCC::STATE_PREAMBLE;
  packet.len = 0;
  packet.preamble = 0;
  byte = 0;
  xor_value = 0;
  bit_count = 0;
}

void motor(DCC::MOTOR movement) {
  switch (movement) {
    case DCC::MOTOR_FWD:
      DL("motor fwd")
      PORT_MOTOR.OUTSET = MOTORIN1;
      PORT_MOTOR.OUTCLR = MOTORIN2;
      break;
    case DCC::MOTOR_REW:
      DL("motor rew")
      PORT_MOTOR.OUTSET = MOTORIN2;
      PORT_MOTOR.OUTCLR = MOTORIN1;
      break;
    case DCC::MOTOR_STOP:
      DL("motor stop")
      PORT_MOTOR.OUT &= ~(MOTORIN1 | MOTORIN2);
      break;
  }
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
        if (xor_value == 0 && (3 <= packet.len) && (packet.len <= 6)) {
          // valid packet, ignore idle packets
          if (!(packet.data[0] == 255 && packet.data[1] == 0 && packet.len == 3)) {
            queue.push(packet);
          }
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
  // ops mode
  if (packet.preamble < 20) {
    uint8_t addr = packet.data[0];

    if (0 == addr) {
      DF("rst p: %u\n", reset_packet_count++);
    } else if ((0 < addr) && (addr <= 127)) {
      DL("multfct dec 7bit n/a");
      uart_arr("  p", packet.data, packet.len);
    } else if ((128 <= addr) && (addr <= 191)) {
      DL("accessory 9/11bit");
      uart_arr("  p", packet.data, packet.len);
      parse_accessory_opmode(packet);
    } else if ((192 <= addr) && (addr <= 231)) {
      DL("multifct dec 14bit n/a");
      uart_arr("  p", packet.data, packet.len);
    } else if ((232 <= addr) && (addr <= 254)) {
      DL("future n/a");
      uart_arr("  p", packet.data, packet.len);
    } /*else if (addr == 255 && packet.data[1] == 0 && packet.len == 3) {
      // idle packet
    };*/
  } else {
    // service mode
    DL("service");
    uart_arr("  p", packet.data, packet.len);
    parse_servicemode(packet);
  }
}

/*
 * basic (decoder addr)  9bit: {preamble} 0 10AAAAAA 0 1aaaCDDR 0 EEEEEEEE 1
 * basic (output addr)  11bit: {preamble} 0 10AAAAAA 0 1aaaCAAR 0 EEEEEEEE 1
 * extended 11bit:             {preamble} 0 10AAAAAA 0 0aaa0AA1 0 DDDDDDDD 0 EEEEEEEE 1
 *
 * A: address bit
 * a: address bit complement
 * C: power (0=inactive, 1=active)
 * D: port (0-3)
 * R: direction (0:left/diverting/stop, 1:right/straight/run)
 *
 * http://normen.railcommunity.de/RCN-213.pdf
 * https://dccwiki.com/Accessory_Decoder_Addressing
 * decoder 1, port 1, 2, 3, 4
 * decoder 2, port 5, 6, 7, 8
 * ...
 * addr 5: (2-1)*4 + 0 + 1
 * -> address (decoder port address: (decoder-1)*4 + port[0-3] + 1
 *  basic: 82 e0 62        10000010 11100000 01100010
 *  extended: 81 63 00 e2  10000001 01100011 00000000 11100010
 */
void parse_accessory_opmode(DCC::PACKET packet) {
  uint8_t p1 = packet.data[1];
  uint8_t power = (p1 & 0x08)>>3; // "C" 0=inactive, 1=active
  uint8_t port = ((p1 & 0x06)>>1); // "DD"
  uint8_t direction = p1 & 0x01; // "R" 0=left/diverting/stop, 1=right/straight/drive
  uint16_t addr_decoder = (packet.data[0] & 0x3f) | ((uint16_t)((~p1 & 0x70))<<2); // 9bit, CV29[6]=0
  uint16_t addr_output = ((((addr_decoder - 1) << 2) | port) + 1); // 11bit, CV29[6]=1

  uint8_t is_basic = packet.len == 3 && (p1 & 0x80) == 0x80;
  uint8_t is_extended = packet.len == 4 && (p1 & 0x89) == 0x01;

  uint16_t addr = (cfg.cv29 & 0x40) ? addr_output : addr_decoder;

  if (is_basic) {
    DL("  basic");
    DF("   9bit: %04u, power: %u, port: %u, dir: %u\n", addr_decoder, power, port, direction);
    DF("  11bit: %04u, power: %u, port: -, dir: %u\n", addr_output, power, direction);
    DF("   addr (%s): %04u\n", (cfg.cv29 & 0x40) ? "output": "decoder", addr);
  } else if (is_extended) {
    DL("  extended");
    DF("  11bit: %04u, data: %u\n", addr_output, packet.data[2]);
  }

  // write new address from command
  if (is_learning && (is_basic || is_extended)) {
    DF("new addr: 0x%04x\n", addr);
    write_cv(1, addr & 0xff, 1); // load config only after fully writing address
    write_cv(9, (addr >> 8) & 0xff);
    toggle_learning_mode(0);
  }

  // check address
  if (addr != cfg.addr) return;

  if (is_basic) {
    if ((cfg.cv29 & 0x40)) {
      process_basic_output(power, direction);
    } else {
      process_basic_decoder(port, power, direction);
    }
  } else if (is_extended) {
    // extended
    process_extended(packet.data[2]);
  } else if (packet.len == 6 && ((p1 & 0x80) == 0x80 || (p1 & 0x89) == 0x89) && (packet.data[2] & 0xf0) == 0xe0) {
    // ops mode configuration
    uint8_t cmd = (packet.data[2] & 0x0c) >> 2;
    uint16_t cv_addr = (((packet.data[2] & 0x03) << 8) | packet.data[3]) + 1;
    parse_cv(DCC::OPS, cmd, cv_addr, packet.data[4]);
  }
}

/*
 * https://dccwiki.com/Service_Mode_Programming
 *
 * {longpreamble} 0 0111CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 */
void parse_servicemode(DCC::PACKET packet) {
  if ( packet.len == 4 && (packet.data[0] & 0xf0) == 0x70) {
    uint8_t cmd = (packet.data[0] & 0x0c) >> 2;
    uint16_t cv_addr = (((packet.data[0] & 0x03) << 8) | packet.data[1]) + 1;
    parse_cv(DCC::SERVICE, cmd, cv_addr, packet.data[2]);
  }
}

/*
 * basic accessory
 * output addressing (11bit)
 *
 * direction (0:left/diverting/stop, 1:right/straight/run)
 */
void process_basic_output(uint8_t power, uint8_t direction) {
  toggle_track(direction);
}

/*
 * basic accessory
 * decoder addressing (9bit)
 */
void process_basic_decoder(uint8_t port, uint8_t power, uint8_t direction) {
  toggle_track(direction);
}

void toggle_track(uint8_t new_position) {
  if (cfg.current_position != new_position) {
    ts = clock.current_tick;
    PORT_LED.OUT |= LED_RED | LED_GREEN;
    motor(new_position == 1 ? DCC::MOTOR_FWD : DCC::MOTOR_REW);
  }
}

/*
 * extended accessory (11bit)
 */
void process_extended(uint8_t data) {
  toggle_track(data ? 1 : 0);
}

/*
 * ack only in service mode
 *
 * https://dccwiki.com/Decoder_Programming
 * TODO "Two identical packets are required before the decoder will act. If one of the packets is corrupted nothing will happen"
 * TODO block address changes: "You cannot change the decoder address in this mode"
 *
 *  ops mode basic:    {preamble} 0 10AAAAAA 0 1aaaCDDR 0 1110KKVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *  ops mode extended: {preamble} 0 10AAAAAA 0 1aaa0AA1 0 1110KKVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *  ops mode (bit):    {preamble}                       0 110110VV 0 VVVVVVVV 0 111KDBBB 0 EEEEEEEE 1
 *
 *  service mode:      {longpreamble}                   0 0111KKVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *  service mode (bit):{longpreamble}                   0 011110VV 0 VVVVVVVV 0 111KDBBB 0 EEEEEEEE 1
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
      if (write_cv(cv_addr, cv_data) == cv_data) {
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

/*
 * https://dccwiki.com/Configuration_Variable
 */
uint8_t read_cv(uint16_t cv_addr) {
  uint8_t index = get_memory_index(cv_addr);
  if (index == 0xff) return index;

  return eeprom_read_byte(&ee_cfg[index]);
}

uint8_t write_cv(uint16_t cv_addr, uint8_t cv_data, uint8_t do_not_load_config) {
  uint8_t update_eeprom = 1;
  uint8_t update_config = 1;

  switch (cv_addr) {
    case 8: // manufacturer id
      factory_default();
      update_eeprom = 0;
      break;
    case 7: // version
      update_config = 0;
      update_eeprom = 0;
      break;
  }

  if (update_eeprom) {
    eeprom_update_byte(&ee_cfg[get_memory_index(cv_addr)], cv_data);
  }

  if (!do_not_load_config && update_config) {
    load_config();
  }
  return read_cv(cv_addr);
}

void ack_cv(DCC::MODE mode) {
  switch (mode) {
    case DCC::SERVICE:
      {
      DL("ack on");
      uint32_t ts_ack = clock.current_tick;
      while (clock.current_tick < (ts_ack + 24)); // 0.006s/(8/32768)
      DL("ack off");
      break;
      }
    case DCC::OPS:
      // no feedback on operation mode
      break;
  }
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
    case  33:
      index = DCC::CV33_POSITION;
      break;
    case 34:
      index = DCC::CV34_DELAY;
      break;
    default:
      DF(NOK("CV not configured: %u") "\n", cv_addr);
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
  cfg.current_position = read_cv(33);
  cfg.delay = read_cv(34) * 82; // 0.02/(8/32768)

  DL("loaded into config:");
  DF("CV29: 0x%02x\n", cfg.cv29);
  DF("addr: %u (0x%04x)\n", cfg.addr, cfg.addr);
  DF("pos: %u\n", cfg.current_position);
  DF("delay: %lums (%u ticks)\n", cfg.delay*8000UL/32768, cfg.delay); //xms =1000*8/32768
}

void toggle_learning_mode(uint8_t force_value) {
  if (force_value) is_learning = force_value;
  else is_learning = !is_learning;

  if (is_learning) {
    PORT_LED.OUT |= LED_RED | LED_GREEN;
  } else {
    PORT_LED.OUTCLR = cfg.current_position ? LED_RED : LED_GREEN;
  }
}

/*
 * isr for DCC signal
 * detects duration of impulse
 */
ISR(TCB0_INT_vect) {
  uint16_t width = TCB0.CCMP; // 0.1us (500 = 50us)

  // 1: 52-64us ±6
  // 0: 116us
  if ((460 <= width) && (width < 700)) process_bit(1);
  else if ((900 <= width) && (width < 10000)) process_bit(0);
  else reset();
}

/*
 * isr to detect if we want to set the decoder in learning mode
 * set PA7 low will toggle it
 */
ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // clear flags

  if (flags & PORT_INT7_bm) {
    // debounce: block contact for some time
    if ((clock.current_tick-ts_debounce) > 4096) { //  1s/(8/32768)
      ts_debounce = clock.current_tick;
      toggle_learning_mode();
    }
  }
}

#ifdef __LIMITSENSORS_ENABLED__
ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; // clear flags

  if (flags & (PORT_INT0_bm | PORT_INT1_bm)) {
    // flags & PORT_INT1_bm -> current_position = 1 else 0
    current_position = flags & PORT_INT1_bm;
    position_reached();
  }
}
#endif

void position_reached() {
  ts = 0;
  // no need to reload all the config here
  write_cv(33, !cfg.current_position, 1);
  cfg.current_position = !cfg.current_position;

  PORT_LED.OUTCLR = cfg.current_position ? LED_RED : LED_GREEN;
  motor(DCC::MOTOR_STOP);
}

/*
 * *********************************************************************************
 * main
 * *********************************************************************************
 */
int main(void) {
  mcu_init();

  // load default cv's if manufacturer id is not "diy"
  if (read_cv(8) != DCC::CV08_MANUFACTURER_DIY) {
    factory_default();
  }
  load_config();

  PORT_LED.DIRSET |= LED_GREEN | LED_RED;

  PORTA.DIRCLR = PIN7_bm;
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

  PORT_LED.OUTSET = cfg.current_position ? LED_GREEN : LED_RED;

  #ifdef __LIMITSENSORS_ENABLED__
    PORTB.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

    if (PORTB.IN & (PIN0_bm | PIN1_bm)) {
      DL("move to home");
      toggle_track(0);
    }
  #endif

  EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc; // ASYNCUSER0 = TCB0, connect with channel 0
  EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN5_gc; // DCC PA5

  TCB0.CTRLB = TCB_CNTMODE_PW_gc; // input capture pulse width
  TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm | TCB_FILTER_bm; // edge: signal inverted
  TCB0.INTCTRL = TCB_CAPT_bm; // enable capture interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  DL("waiting for data...");

  DCC::PACKET p;
  while (1) {
    // keep this here instead of toggle_track, so we have a backup to stop even if limit sensor active
    if (ts && (clock.current_tick - ts) > cfg.delay) {
      // toggling done
      position_reached();
    }

    // if nothing in progress, we can fetch the next command
    // unless we have limit sensors: then we can interrupt directly
    // TODO might not work, as we change current_position only when reached
    #ifdef __LIMITSENSORS_ENABLED__
    if (!ts) {
    #endif
      queue.pull(p);
      if (p.len) {
        parse_packet(p);
      }
    #ifdef __LIMITSENSORS_ENABLED__
    }
    #endif
  }
}
