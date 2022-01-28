/*
 * creates a DCC signal (simulator)
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"

#include "string.h"

volatile uint8_t done = 0;
char buff[20];
char uart_data[20];
volatile uint8_t buff_p = 0;
namespace CMD_STATE {
  typedef enum {
    NONE,
    TURN_LEFT,
    TURN_RIGHT,
    CHECK,
    TOGGLE_IDLE,
    TOGGLE_EXTENDED,
    TOGGLE_PRG_CMD_TYPE,
    TOGGLE_PRG_MODE,
    RESET,
    CMD_ADDR_WAITFOR,
    CMD_CHGADDR_WAITFOR,
    CMD_CHGADDR_PROCESS,
    CMD_CV_WAITFOR,
    CMD_CV_PROCESS,
    CMD_CVB_WAITFOR,
    CMD_CVB_PROCESS,
    SHOW_HELP,
  } Type_Cmd_State;
}
CMD_STATE::Type_Cmd_State cmd_state = CMD_STATE::SHOW_HELP;

namespace PRG {
  typedef enum {
    SERVICE = 0,
    OPS = 1,
  } Type_Prg_Mode;

  typedef enum {
    RESERVED = 0,
    READ = 0b01,
    WRITE = 0b11,
    BIT = 0b10,
  } Type_Prg_Cmd_Type;
}


pins_t R = PB6; // also control output
pins_t L = PB7;
uint16_t EEMEM ee_decoder_addr;
volatile uint16_t decoder_addr;

/*
 * get <num_of_chars> chars before current pointer of ring buffer
 */
uint8_t get_prev_chars(char *buff, char *data, uint8_t num_of_chars) {
  uint8_t len_buff = strlen(buff);
  num_of_chars = num_of_chars > len_buff ? len_buff : num_of_chars;
  uint8_t pos = buff_p;
  for (uint8_t i=0; i<num_of_chars; i++) {
    uart_rollbefore(&pos, 20);
    data[num_of_chars-1-i] = buff[pos];
  }
  data[num_of_chars] = '\0';

  return num_of_chars;
}

uint16_t _char2int(char *data_in, uint8_t len) {
  uint16_t value = 0;
  uint8_t base = 10;
  uint8_t start = 0;
  // if addr_char 0x1234 -> hex
  if (data_in[1] == 'x') {
    base = 16;
    start = 2;
  }
  // if addr_char 0b10101 -> binary
  else if (data_in[1] == 'b') {
    base = 2;
    start = 2;
  }
  for (uint8_t i=start; i<len; i++) {
    value = value*base + data_in[i] - '0';
  }

  return value;
}

/*
 * convert char to uint
 * eg 123, 0x123, 0b001
 */
uint16_t buff2int(char *buff) {
  char data_in[19];
  uint8_t l = get_prev_chars(buff, data_in, 18);
  return _char2int(data_in, l);
}

uint16_t char2int(char *buff) {
  return _char2int(buff, strlen(buff));
}

/*
 * isr for uart read
 */
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  if (in != '\n') {
    // backspace
    if (in == '\r') {
      uart_rollbefore(&buff_p, 20);
    }
    // any key
    else {
      buff[buff_p] = in;
      uart_rollover(&buff_p, 20);
    }
  }

  if (cmd_state == CMD_STATE::NONE) {
    switch (in) {
      case '0'+CMD_STATE::TURN_LEFT : cmd_state = CMD_STATE::TURN_LEFT; break;
      case '0'+CMD_STATE::TURN_RIGHT: cmd_state = CMD_STATE::TURN_RIGHT; break;
      case '0'+CMD_STATE::CHECK: cmd_state = CMD_STATE::CHECK; break;
      case '0'+CMD_STATE::TOGGLE_IDLE: cmd_state = CMD_STATE::TOGGLE_IDLE; break;
      case '0'+CMD_STATE::TOGGLE_EXTENDED: cmd_state = CMD_STATE::TOGGLE_EXTENDED; break;
      case '0'+CMD_STATE::TOGGLE_PRG_CMD_TYPE: cmd_state = CMD_STATE::TOGGLE_PRG_CMD_TYPE; break;
      case '0'+CMD_STATE::TOGGLE_PRG_MODE: cmd_state = CMD_STATE::TOGGLE_PRG_MODE; break;
      case '0'+CMD_STATE::RESET: cmd_state = CMD_STATE::RESET; break;
    }
  }

  if (in == '\n') {
    if (cmd_state == CMD_STATE::NONE) {
      if (get_prev_chars(buff, uart_data, 7) && !strcmp(uart_data, "chgaddr")) {
        cmd_state = CMD_STATE::CMD_CHGADDR_WAITFOR;
        D("enter new addr for device: ");
      }
      else if (get_prev_chars(buff, uart_data, 4) && !strcmp(uart_data, "addr")) {
        cmd_state = CMD_STATE::CMD_ADDR_WAITFOR;
        D("enter addr cmds are sent to: ");
      }
      // 1234[w|r]0b00000000
      else if (get_prev_chars(buff, uart_data, 2) && !strcmp(uart_data, "cv")) {
        cmd_state = CMD_STATE::CMD_CV_WAITFOR;
        D("enter CV cmd <cv addr>[r|w]<cv value> (eg 121r0x05): ");
      }
      // 1234[w|r][0-7][0|1]
      else if (get_prev_chars(buff, uart_data, 3) && !strcmp(uart_data, "cvb")) {
        cmd_state = CMD_STATE::CMD_CVB_WAITFOR;
        D("enter CV bit cmd <cv addr>[r|w]<bitpos><bitvalue> (eg 121r71): ");
      }
      else if (get_prev_chars(buff, uart_data, 4) && !strcmp(uart_data, "help")) {
        cmd_state = CMD_STATE::SHOW_HELP;
      }
    }

    else if (cmd_state == CMD_STATE::CMD_ADDR_WAITFOR) {
      uint16_t addr = buff2int(buff);
      eeprom_update_word(&ee_decoder_addr, addr);
      decoder_addr = addr;
      DF("address in use: %u (0x%04x)\n", decoder_addr, decoder_addr);
      cmd_state = CMD_STATE::NONE;
    }

    else if (cmd_state == CMD_STATE::CMD_CV_WAITFOR) {
      get_prev_chars(buff, uart_data, 20);
      cmd_state = CMD_STATE::CMD_CV_PROCESS;
    }

    else if (cmd_state == CMD_STATE::CMD_CVB_WAITFOR) {
      get_prev_chars(buff, uart_data, 20);
      cmd_state = CMD_STATE::CMD_CVB_PROCESS;
    }

    else if (cmd_state == CMD_STATE::CMD_CHGADDR_WAITFOR) {
      get_prev_chars(buff, uart_data, 20);
      cmd_state = CMD_STATE::CMD_CHGADDR_PROCESS;
    }

    // clear ring buffer
    buff_p = 0;
    for (uint8_t ii=0; ii<8; ii++) buff[ii] = 0;
  }
}

/*
 * timer timeout
 */
ISR(TCA0_OVF_vect) {
  done = 1;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}


void timer_init() {
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
}

void send_bit(uint8_t length) {
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV1_gc;
  pins_set(&R, 1);
  pins_set(&L, 0);
  done = 0;
  TCA0.SINGLE.PER = length ? 285 : 850; // 10MHz*58us = 580
  TCA0.SINGLE.CNT = 0;
  while (!done); // wait for interrupt
  pins_set(&R, 0);
  pins_set(&L, 1);
  done = 0;
  TCA0.SINGLE.PER = length ? 395 : 980;
  TCA0.SINGLE.CNT = 0;
  while (!done); // wait for interrupt
  pins_set(&R, 1);
  pins_set(&L, 1);
  TCA0.SINGLE.CTRLA = 0;
}

void send_byte(uint8_t data) {
  for (uint8_t c=0; c<8; c++) {
    send_bit(data & (1<<(7-c)));
  }
}

/*
 * length: 3-6bytes (inc last xor byte)
 * turnout: 9bit address (accessory decoder 9bit address)
 * signal: 11bit address (accessory decoder extended 14bit address)
 *
 * https://dccwiki.com/Address_Range
 *
 * https://www.opendcc.de/elektronik/opendcc/opendcc_sw_lenz.html
 * addr 0      : broadcast for locos
 * addr 1-127  : addr starts with 0 -> loco cmd with short addr
 * addr 128-191: accessory decoders
 * addr 192-231: loco long addr, following addrbits in next byte
 *
 * example loco short addr:
 * addr loco: 0AAAAAAA -> realaddr & 0x7F
 * data loco: 01DCSSSS -> 0x40 | (speed & 0x0F) | (direction << 5)
 *            C   : light
 *            SSSS: speed 0-14
 *            D   : direction
 *
 * example track switch / turnout (aka Weiche):
 * addr track: 10AAAAAA -> 0x80 + (realaddr & 0x3F)
 * data track: 1AAA1BBR -> 0x80 + (realaddr / 0x40) ^ 0x07) * 0x10
 *             A8-A6: are inverted
 *             BB   : local addr on the decoder (0, 1, 2, 3)
 *             R    : output bit
 *
 * we only support 3bytes (addr, data, xor)
 */
void send_packet(uint8_t *packets, uint8_t len, PRG::Type_Prg_Mode mode = PRG::OPS) {
  // preamble
  for (uint8_t c=0; c<(mode == PRG::OPS ? 12 : 20); c++) send_bit(1);

  // data
  uint8_t x = 0;
  for (uint8_t c=0; c<len; c++) {
    send_bit(0);
    send_byte(packets[c]);
    x ^= packets[c];
  }
  send_bit(0);
  send_byte(x);
  send_bit(1);
  pins_set(&R, 1);
  pins_set(&L, 1);
  // uart_arr("packets", packets, len);
  // DF("xor: 0x%02x\n", x);
  // DF("a: 0x%02x, d: 0x%02x, c: 0x%02x\n", addr, data, addr^data);
}

/*
 * convert eg turnout address to module address and port
 * (for basic accessories)
 * see https://wiki.rocrail.net/doku.php?id=addressing:accessory-pg-de
 * addr starts at 1
 * module starts at 1 (except for lenz / roco it starts at 0)
 */
void split_addr(uint16_t addr, uint8_t *module_addr, uint8_t *port, uint8_t is_roco) {
  *module_addr = (addr - 1) / 4 + (is_roco ? 0 : 1);
  *port = (addr - 1) % 4 + 1;
}

/*
 * Basic accessory decoder 11bit address
 * {preamble} 0 10AAAAAA 0 1AAACDDR 0 EEEEEEEE 1
 * example  weiche / turnout / track switch
 *
 * weiche 5 = module addr (AAAAAAAAA): 2, port (DD): 1 (port 1-4 are sent as 0-3)
 *            0 10000010 0 11110001 (0x82 0xf0)
 *
 * each module_addr (9bit) has 4 ports (1-4)
 */
void basic_accessory(uint16_t addr, uint8_t activation, uint8_t output, uint8_t is_roco = 0) {
  uint8_t packets[2];
  uint8_t module_addr;
  uint8_t port;
  split_addr(addr, &module_addr, &port, is_roco);

  packets[0] = 0x80 | (module_addr & 0x3f);
  packets[1] = 0x80 | ((~module_addr & 0x1c0)>>2) | (((port - 1) & 0x03)<<1) | (activation ? 0x01<<3 : 0) | (output ? 0x01 : 0);
  pins_set(&PA7, 0);
  send_packet(packets, 2);
  pins_set(&PA7, 1);
  // uart_arr("packets", packets, 2);
}

/*
 * accessory decoder configuration variable access
 * basic/ops     {preamble} 0 10AAAAAA 0 1AAA1AA0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 * basic/service {preamble} 0                       0111CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *
 * CC: 00=reserved, 01=verify byte, 11=writy byte, 10=bit manipulation
 * VVVVVVVVVV: 10bit address for CV1-1024
 * DDDDDDDD: data of CV
 *
 * cv_addr: 1-1024
 * mode:     PRG::OPS, PRG::SERVICE
 * cmd_type: PRG::READ, PRG::WRITE, PRG::BIT
 * addr is ignored if PRG::SERVICE
 */
void basic_accessory_prg(uint16_t addr, PRG::Type_Prg_Mode mode, PRG::Type_Prg_Cmd_Type cmd_type, uint16_t cv_addr, uint8_t cv_data, uint8_t skip_resets = 0, uint8_t is_roco = 0) {
  uint8_t packets[5] = {0};
  uint8_t module_addr;
  uint8_t port;
  uint8_t index = 0;
  uint8_t num = 3;
  uint8_t cmd_start = 0b0111<<4; // service

  // send reset packets 1st (only in service mode for before 1st packet)
  if (mode == PRG::SERVICE && !skip_resets) {
    for (uint8_t c=0; c<25; c++) {
      send_packet(packets, 2);
    }
  }

  cv_addr -= 1; // input: 1-1024
  split_addr(addr, &module_addr, &port, is_roco);
  if (mode == PRG::OPS) {
    packets[0] = 0x80 | (module_addr & 0x3f);
    packets[1] = 0x88 | ((~module_addr & 0x1c0)>>2) | (((port - 1) & 0x03)<<1);
    index += 2;
    num += 2;
    cmd_start = 0b1110<<4;
  }

  packets[index] = cmd_start | (cmd_type<<2) | ((cv_addr & 0x300)>>8);
  packets[index+1] = 0xff & cv_addr;
  packets[index+2] = cv_data;
  pins_set(&PA7, 0);
  send_packet(packets, num, mode);
  // write requires 2 similar packets while in ops mode (in real env)
  if (mode == PRG::OPS && cmd_type == PRG::WRITE) send_packet(packets, num, mode);
  pins_set(&PA7, 1);
  // uart_arr("prg", packets, num);
}

void basic_accessory_prg_bit(uint16_t addr, PRG::Type_Prg_Mode mode, uint16_t cv_addr, uint8_t write, uint8_t bit_addr, uint8_t bit_value, uint8_t is_roco = 0) {
  uint8_t cv_data = 0xe0 | (write ? 0x10 : 0x00) | (bit_value ? 0x08 : 0x00) | (0x07 & bit_addr);
  basic_accessory_prg(addr, mode, PRG::BIT, cv_addr, cv_data, 0, is_roco);
}

/*
 * extended accessory decoder 11bit address
 * {preamble} 0 10AAAAAA 0 0AAA0AA1 0 000XXXXX 0 EEEEEEEE 1
 */
void extended_accessory(uint16_t addr, uint8_t output) {
  uint8_t packets[3];
  packets[0] = 0x80 | ((addr & 0xfc)>>2);
  packets[1] = 0x01 | ((~addr & 0x700)>>4) | ((addr & 0x03)<<1);
  packets[2] = output;
  pins_set(&PA7, 0);
  send_packet(packets, 3);
  pins_set(&PA7, 1);
}

/*
 * extended accessory decoder configuration variable access
 * basic/ops     {preamble} 0 10AAAAAA 0 0AAA0AA1 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 * basic/service {preamble} 0                       0111CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *
 * CC: 00=reserved, 01=verify byte, 11=writy byte, 10=bit manipulation
 * VVVVVVVVVV: 10bit address for CV1-1024
 * DDDDDDDD: data of CV
 *
 * cv_addr: 1-1024
 * mode:     PRG::OPS, PRG::SERVICE
 * cmd_type: PRG::READ, PRG::WRITE, (not yet supported: PRG::BIT)
 * addr is ignored if PRG::SERVICE
 */
void extended_accessory_prg(uint16_t addr, PRG::Type_Prg_Mode mode, PRG::Type_Prg_Cmd_Type cmd_type, uint8_t cv_addr, uint8_t cv_data) {
  uint8_t packets[5] = {0};
  uint8_t index = 0;
  uint8_t cmd_start = 0b0111<<4;

  cv_addr -= 1; // 1-1024
  if (mode == PRG::OPS) {
    packets[0] = 0x80 | ((addr & 0xfc)>>2);
    packets[1] = 0x88 | ((~addr & 0x700)>>4) | ((addr & 0x03)<<1);
    index += 2;
    cmd_start = 0b1110<<4;
  }

  packets[index] = cmd_start | (cmd_type<<2) | ((cv_addr & 0x300)>>8);
  packets[index + 1] = 0xff & cv_addr;
  packets[index + 2] = cv_data;
  pins_set(&PA7, 0);
  send_packet(packets, 5);
  pins_set(&PA7, 1);
}

/*
 * Multifunction decoder (extended)
 * (locomotives, ...)
 *
 * type see https://dccwiki.com/Digital_Packet
 * b000 decoder and consist control instructions
 * b001 advanced operations
 * ...
 *
 * only supports one instruction byte (extended address mode 14bit address)
 * 0x0000 - 0x3fff (0xc00 - 0xfff)
 * {preamble} 0 11AAAAAA 0 AAAAAAAA 0 CCCDDDDD 0 EEEEEEEE 1
 * A: address bit
 * C: type bit
 * D: data bit
 * E: control bit (xor)
 */
void multifunction(uint16_t addr, uint16_t type, uint8_t data) {
  uint8_t packets[3];
  packets[0] = (0xc0 | (addr >> 8)); // packet must be 0b11xxxxxx
  packets[1] = addr & 0xff;
  packets[2] = ((type & 0x07)<<5) | (data & 0x1f);
  pins_set(&PA7, 0);
  send_packet(packets, 3);
  pins_set(&PA7, 1);
  // uart_arr("multi", packets, 3);
}

void bitwise(char *buf, uint8_t value) {
  for (uint8_t i=0; i<8; i++) {
    buf[i] = value & 0x80 ? '1' : '0';
    value <<= 1;
  }
}

void idle_packet() {
  uint8_t packets[2];
  packets[0] = 0xff;
  packets[1] = 0x00;
  send_packet(packets, 2); // idle packet
}

void print_outputs(uint8_t port_outputs) {
  char buf[9];
  bitwise(buf, port_outputs);
  buf[8] = '\0';
  DF("outputs: %s\n", buf);
}

int main(void) {
  mcu_init(1);


  decoder_addr = eeprom_read_word(&ee_decoder_addr);
  if (decoder_addr == 0xffff) decoder_addr = 0x05; // default 267=0x10b
  DF("address in use: %u\n", decoder_addr);

  pins_output(&L, 1);
  pins_output(&R, 1);
  pins_set(&R, 1);
  pins_set(&L, 0);

  pins_output(&PA7, 1);
  pins_set(&PA7, 0);

  uint8_t port_outputs = 0;
  uint8_t fill_with_idle = 0;
  uint8_t is_extended_mode = 0;
  PRG::Type_Prg_Cmd_Type prg_cmd_type = PRG::READ;
  PRG::Type_Prg_Mode prg_mode = PRG::SERVICE;

  timer_init();

  while  (1) {
    switch (cmd_state) {
      case CMD_STATE::TURN_LEFT:
        if (is_extended_mode) {
          extended_accessory(decoder_addr, 0);
        } else {
          basic_accessory(decoder_addr, 0, 0, 0); // addr, activation, output, is_roco
        }
        port_outputs &= !(1 << 7); // clear bit 7
        if (!fill_with_idle) print_outputs(port_outputs); // avoid times without packets
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TURN_RIGHT:
        if (is_extended_mode) {
          extended_accessory(decoder_addr, 1);
        } else {
          basic_accessory(decoder_addr, 0, 1, 0);
        }
        port_outputs |= (1 << 7); // set bit 7
        if (!fill_with_idle) print_outputs(port_outputs);
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_IDLE:
        fill_with_idle = !fill_with_idle;
        DF("fill with idle: %s\n", fill_with_idle ? "yes" : "no");
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_EXTENDED:
        is_extended_mode = !is_extended_mode;
        DF("mode: %s\n", is_extended_mode ? "extended" : "basic");
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_PRG_CMD_TYPE:
        prg_cmd_type = prg_cmd_type == PRG::READ ? PRG::WRITE : PRG::READ;
        DF("prg type: %s\n", prg_cmd_type == PRG::READ ? "read" : "write");
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_PRG_MODE:
        prg_mode = prg_mode == PRG::OPS ? PRG::SERVICE : PRG::OPS;
        DF("prg mode: %s\n", prg_mode == PRG::OPS ? "ops" : "service");
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::RESET:
        {
          // 25 reset packets
          uint8_t packets[2] = {0};
          pins_set(&PA7, 0);
          for (uint8_t c=0; c<25; c++) {
            send_packet(packets, 2);
          }
          pins_set(&PA7, 1);
          cmd_state = CMD_STATE::NONE;
        }
        break;
      case CMD_STATE::CHECK:
        {
          basic_accessory_prg(decoder_addr, prg_mode, prg_cmd_type, 121, 0x05);
          if (!fill_with_idle) DF("prg cmd: %s - %s\n", prg_mode == PRG::OPS ? "ops" : "service", prg_cmd_type == PRG::READ ? "read" : "write");
          cmd_state = CMD_STATE::NONE;
        }
        break;
      case CMD_STATE::NONE:
        break;
      case CMD_STATE::CMD_ADDR_WAITFOR:
      case CMD_STATE::CMD_CV_WAITFOR:
      case CMD_STATE::CMD_CVB_WAITFOR:
      case CMD_STATE::CMD_CHGADDR_WAITFOR:
        // handled in ISR
        break;
      case CMD_STATE::CMD_CHGADDR_PROCESS:
        {
          uint16_t addr = char2int(uart_data);
          basic_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 121, addr & 0xff); // lsb
          // wait 100ms filling with reset
          uint32_t start = clock.current_tick;
          uint8_t packets[2] = {0};
          while ((clock.current_tick - start) < 410) {
            send_packet(packets, 2);
          }
          basic_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 120, (addr >> 8) & 0xff, 1); // msb, skip intro reset packets
          DF("new addr: %u/0x%03x\n", addr, addr);
          cmd_state = CMD_STATE::NONE;
        }
        break;
      case CMD_STATE::CMD_CV_PROCESS:
        {
          // uart_data = "121r0x05" cv121 compare with 0x05
          char cv_addr_char[5] = {0};
          uint8_t cv_addr_pos = 0;
          uint8_t rw_seen = 0;
          char cv_value_char[10] = {0};
          uint8_t cv_value_pos = 0;
          PRG::Type_Prg_Cmd_Type cmd_type = PRG::RESERVED;
          for (uint8_t i=0; i<strlen(uart_data); i++) {
            if (uart_data[i] == 'r' || uart_data[i] == 'w') {
              rw_seen = 1;
              cmd_type = uart_data[i] == 'r' ? PRG::READ : PRG::WRITE;
            } else if (rw_seen == 0) {
              cv_addr_char[cv_addr_pos++] = uart_data[i];
            } else {
              cv_value_char[cv_value_pos++] = uart_data[i];
            }
          }

          uint16_t cv_addr = char2int(cv_addr_char);
          uint8_t cv_value = char2int(cv_value_char);
          DF("%s cv#%u: 0x%02x\n", cmd_type == PRG::WRITE ? "write" : "read", cv_addr, cv_value);
          basic_accessory_prg(decoder_addr, prg_mode, cmd_type, cv_addr, cv_value);

          cmd_state = CMD_STATE::NONE;
        }
        break;
      case CMD_STATE::CMD_CVB_PROCESS:
        {
          // uart_data = "121r70" cv121 compare bit 7 with 0
          char cv_addr_char[5] = {0};
          uint8_t cv_addr_pos = 0;
          uint8_t rw_seen = 0;
          uint8_t bit_addr = 0;
          uint8_t bit_value = 0;
          uint8_t write = 0;
          for (uint8_t i=0; i<strlen(uart_data); i++) {
            if (uart_data[i] == 'r' || uart_data[i] == 'w') {
              rw_seen = 1;
              write = uart_data[i] == 'w' ? 1 : 0;
            } else if (rw_seen == 0) {
              cv_addr_char[cv_addr_pos++] = uart_data[i];
            } else {
              bit_addr = uart_data[i] - '0';
              bit_value = uart_data[i+1] - '0';
              break;
            }
          }
          uint16_t cv_addr = char2int(cv_addr_char);
          DF("%s cv#%u: bit %u, value %u\n", write == 1 ? "write" : "read", cv_addr, bit_addr, bit_value);
          basic_accessory_prg_bit(decoder_addr, prg_mode, cv_addr, write, bit_addr, bit_value);

          cmd_state = CMD_STATE::NONE;
        }
        break;
      case CMD_STATE::SHOW_HELP:
        DL("usage:");
        DF("  %u : turn left\n", CMD_STATE::TURN_LEFT);
        DF("  %u : turn right\n", CMD_STATE::TURN_RIGHT);
        DF("  %u : check cv\n", CMD_STATE::CHECK);
        DF("  %u : " WARN("%s") " idle packets in between (toggle)\n", CMD_STATE::TOGGLE_IDLE, fill_with_idle ? "yes" : "no");
        DF("  %u : " WARN("%s") " mode (toggle)\n", CMD_STATE::TOGGLE_EXTENDED, is_extended_mode ? "extended" : "basic");
        DF("  %u : " WARN("%s") " prg type (toggle)\n", CMD_STATE::TOGGLE_PRG_CMD_TYPE, prg_cmd_type == PRG::READ ? "read" : "write");
        DF("  %u : " WARN("%s") " prg mode (toggle)\n", CMD_STATE::TOGGLE_PRG_MODE, prg_mode == PRG::OPS ? "ops" : "service");
        DF("  %u : 25 reset packets\n", CMD_STATE::RESET);
        DF("  addr: " WARN("%u/0x%03x") " address cmds are sent to\n", decoder_addr, decoder_addr);
        DL("  chgaddr: change device addr");
        DL("  cv  : read/write cv");
        DL("  cvb : read/write cv bit");
        DL("  help: show this help");
        cmd_state = CMD_STATE::NONE;
        break;
    }

    if (fill_with_idle) {
      idle_packet();
    }
  }
}
