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
volatile CMD_STATE::Type_Cmd_State cmd_state = CMD_STATE::SHOW_HELP;

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

// predefine idle_packets
#define IDLE_PACKETS_SIZE_BYTES 5
uint8_t idle_packets[IDLE_PACKETS_SIZE_BYTES] = { 0xff, 0xf7, 0xf8, 0x01, 0xff };
uint8_t fill_with_idle = 0;
volatile int8_t idle_packets_p = -1;

pins_t R = PB6; // also control output
pins_t L = PB7;
volatile uint8_t output = 0;
uint16_t EEMEM ee_decoder_addr;
volatile uint16_t decoder_addr;

// 6 packets, 20bits preamble = 68bits
// 0=long (116us) , 1=short (58us)
// we create a bit buffer of uint8_t's with size: BUFF_SIZE_BYTES * 8
#define BUFF_SIZE_BYTES 200
uint8_t buffer[BUFF_SIZE_BYTES] = {0};
uint16_t buff_pos_in = 0;
volatile uint16_t buff_pos_out = 0; // max 20*8
volatile uint8_t current_bit = 0;
volatile uint8_t edgecount = 0;

/*
 * get current bit buffer to send out
 * (pointer buff_pos_out)
 */
uint8_t buff_get_current_bit(uint8_t *b, uint16_t p) {
  return (b[p/8] & (1<<(7-(p%8)))) ? 1 : 0;
}

/*
 * increment value and rollover if equals max
 */
uint8_t inc_rollover(volatile uint16_t *value, uint16_t max, uint8_t increment = 1) {
  uint16_t new_value = *value + increment;
  uint8_t overflow = new_value >= max;
  *value = overflow ? new_value - max : new_value;
  return overflow;
}

uint8_t inc_rollover(uint16_t *value, uint16_t max, uint8_t increment = 1) {
  uint16_t new_value = *value + increment;
  uint8_t overflow = new_value >= max;
  *value = overflow ? new_value - max : new_value;
  return overflow;
}

/*
 * add bit to transmission buffer
 * (buff_add_bit)
 */
uint16_t send_bit(uint8_t bit, uint16_t buff_pos_start = 65535) {
  uint8_t bp = buff_pos_start != 65535;
  uint16_t buff_pos = bp ? buff_pos_start : buff_pos_in;

  uint8_t v = 1<<(7-(buff_pos%8));
  if (bit) {
    buffer[buff_pos/8] |= v;
  } else {
    buffer[buff_pos/8] &= ~v;
  }
  uint16_t next = buff_pos;
  inc_rollover(&next, BUFF_SIZE_BYTES*8);
  // wait for free space
  // while (next == buff_pos_out);

  uint8_t warned = 0;
  while (next == buff_pos_out) {
    if (!warned) {
      warned = 1;
      DL("buf ofv");
    }
  }

  if (!bp) buff_pos_in = next;

  return next;
}

/*
 * add byte to transmission buffer
 * (buff_add_byte)
 */
uint16_t send_byte(uint8_t value, uint16_t buff_pos_start = 65535) {
  uint8_t bp = buff_pos_start != 65535;
  uint16_t buff_pos = bp ? buff_pos_start : buff_pos_in;
  uint16_t next = buff_pos;
  uint8_t overflow = inc_rollover(&next, BUFF_SIZE_BYTES*8, 8);
  // wait for free space in buffer (byte)
  // while (overflow && (next >= buff_pos_out));

  uint8_t warned = 0;
  while (overflow && (next >= buff_pos_out)) {
    if (!warned) {
      warned = 1;
      DL("buf ofv");
    }
  }

  uint16_t cur_buff_byte = buff_pos/8;
  uint8_t buff_pos_in_byte = buff_pos%8;
  buffer[cur_buff_byte] &= ~(0xff>>buff_pos_in_byte);
  buffer[cur_buff_byte] |= value>>buff_pos_in_byte;
  if (buff_pos_in_byte) {
    inc_rollover(&cur_buff_byte, BUFF_SIZE_BYTES);
    buffer[cur_buff_byte] &= ~((0xff>>(8-buff_pos_in_byte))<<(8-buff_pos_in_byte));
    buffer[cur_buff_byte] |= (value & (0xff>>(8-buff_pos_in_byte)))<<(8-buff_pos_in_byte);
  }
  inc_rollover(&buff_pos, BUFF_SIZE_BYTES*8, 8);

  if (!bp) buff_pos_in = buff_pos;

  return buff_pos;
}

/*
 * interrupt set to 58us
 * handle 1=58us and 0=116us bit signals from our buffer
 */
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;

  // no data to send
  if (!fill_with_idle && buff_pos_in == buff_pos_out) {
    PORTB.OUT |= PIN6_bm | PIN7_bm; // PB6=PB7=1
    edgecount = 0;
    return;
  }

  // let's toggle as fast as possible in isr
  if (edgecount == 0 || current_bit == 1 || (current_bit == 0 && (edgecount == 2 || edgecount == 4))) {
    uint8_t portvalues = PORTB.IN;
    // clear bit 7, copy pin 6 to 7, then toggle pin 6
    portvalues = (portvalues & ~PIN7_bm) | ((portvalues & PIN6_bm)>>PIN6_bp<<PIN7_bp);
    portvalues ^= PIN6_bm;
    PORTB.OUT = portvalues;
  }

  // signal start
  if (edgecount == 0) {
    uint8_t has_data = (buff_pos_in != buff_pos_out);
    // activate sending idle packets
    if (!has_data && idle_packets_p == -1) {
      idle_packets_p = 0;
    }
    // inactivate sending idle packets
    else if (has_data && idle_packets_p == 0) {
      idle_packets_p = -1;
    }

    // get next bit (from available data buffer or idle buffer)
    if (idle_packets_p == -1) {
      current_bit = buff_get_current_bit(buffer, buff_pos_out);
    } else {
      current_bit = buff_get_current_bit(idle_packets, idle_packets_p);
    }
  }

  edgecount++;

  // signal done
  if ((current_bit == 1 && edgecount == 2) || (current_bit == 0 && edgecount == 4)) {
    // get next bit within it buffer is 0 1 2 3 4 5 6 7 8 9...
    //                                 |    buff[0]    | buff[1]...
    if (idle_packets_p == -1) {
      inc_rollover(&buff_pos_out, BUFF_SIZE_BYTES*8);
    }
    else {
      idle_packets_p = ++idle_packets_p >= (IDLE_PACKETS_SIZE_BYTES*8) ? 0 : idle_packets_p; // make faster than inc_rollover(&idle_packets_p, 40);
    }
    edgecount = 0;
  }
}

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

void timer_init() {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV1_gc;
  TCA0.SINGLE.PER = 580;
  TCA0.SINGLE.CNT = 0;
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
  // we "activate" the new buffer entries only at the end
  uint16_t p;

  // preamble (ops: 12bits, service: 20bits)
  p = send_byte(0xff, buff_pos_in);
  if (mode == PRG::SERVICE) p = send_byte(0xff, p);
  for (uint8_t c=0; c<4; c++) p = send_bit(1, p);

  // data
  uint8_t x = 0;
  for (uint8_t c=0; c<len; c++) {
    p = send_bit(0, p);
    p = send_byte(packets[c], p);
    x ^= packets[c];
  }
  p = send_bit(0, p);
  p = send_byte(x, p);
  p = send_bit(1, p);

  // activate new buffer entries
  buff_pos_in = p;

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
  send_packet(packets, 2);
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

  // send reset packets 1st (only in service mode before 1st packet)
  if (mode == PRG::SERVICE && !skip_resets) {
    for (uint8_t c=0; c<25; c++) {
      send_packet(packets, 2);
    }
  }

  // add address packets
  split_addr(addr, &module_addr, &port, is_roco);
  if (mode == PRG::OPS) {
    packets[0] = 0x80 | (module_addr & 0x3f);
    packets[1] = 0x88 | ((~module_addr & 0x1c0)>>2) | (((port - 1) & 0x03)<<1);
    index += 2;
    num += 2;
    cmd_start = 0b1110<<4;
  }

  cv_addr -= 1; // input: 1-1024
  packets[index] = cmd_start | (cmd_type<<2) | ((cv_addr & 0x300)>>8);
  packets[index+1] = 0xff & cv_addr;
  packets[index+2] = cv_data;
  send_packet(packets, num, mode);
  // write requires 2 similar packets while in ops mode (in real env)
  if (mode == PRG::OPS && cmd_type == PRG::WRITE) send_packet(packets, num, mode);
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
  send_packet(packets, 3);
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
  send_packet(packets, 5);
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
  send_packet(packets, 3);
  // uart_arr("multi", packets, 3);
}

void bitwise(char *buf, uint8_t value) {
  for (uint8_t i=0; i<8; i++) {
    buf[i] = value & 0x80 ? '1' : '0';
    value <<= 1;
  }
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

  PORTB.DIR |= PIN6_bm | PIN7_bm;
  PORTB.OUT |= PIN6_bm | PIN7_bm; // PB6=PB7=1

  /*
  // debug pin for oscilloscope
  pins_output(&PA7, 1);
  pins_set(&PA7, 1);
  */

  uint8_t port_outputs = 0;
  uint8_t is_extended_mode = 0;
  PRG::Type_Prg_Cmd_Type prg_cmd_type = PRG::READ;
  PRG::Type_Prg_Mode prg_mode = PRG::SERVICE;

  timer_init();
  _delay_ms(1);

  // DBG
  /*
    for (uint8_t i=0; i<10; i++) {
      buffer[5*i] = 0xff;
      buffer[5*i+1] = 0xf0;
      buffer[5*i+2] = 0x00;
      buffer[5*i+3] = 0x00;
      buffer[5*i+4] = 0x01;
    }
    buff_pos_in = 400;
    DF("buff_pos_in: %u\n", buff_pos_in);
    uart_arr("buff", buffer, 50);
    while(1);

    uint16_t p = buff_pos_in;
    uint8_t packets[2] = {0};
    uint8_t len = 2;

    // preamble (ops: 12bits, service: 20bits)
    for (uint8_t i=0; i<10; i++) {
      p = send_byte(0xff, p);
      for (uint8_t c=0; c<4; c++) p = send_bit(1, p);

      // data
      uint8_t x = 0;
      for (uint8_t c=0; c<len; c++) {
        p = send_bit(0, p);
        p = send_byte(packets[c], p);
        x ^= packets[c];
      }
      p = send_bit(0, p);
      p = send_byte(x, p);
      p = send_bit(1, p);

      // activate new buffer entries
    }
      buff_pos_in = p;
    DF("buff_pos_in: %u p: %u\n", buff_pos_in, p);
    uart_arr("buff", buffer, 50);

    while(1);
  */
  // DBG

  while  (1) {
    switch (cmd_state) {
      case CMD_STATE::TURN_LEFT:
        if (is_extended_mode) {
          extended_accessory(decoder_addr, 0);
        } else {
          // DF("in: %u out: %u\n", buff_pos_in, buff_pos_out);
          basic_accessory(decoder_addr, 0, 0, 0); // addr, activation, output, is_roco
          // uart_arr("buffer", buffer, BUFF_SIZE_BYTES);
          // DF("in: %u out: %u\n", buff_pos_in, buff_pos_out);
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
          uint8_t packets[2] = {0};
          /*
          DF("in: %u out: %u\n", buff_pos_in, buff_pos_out);
          send_packet(packets, 2);
          uart_arr("buffer", buffer, BUFF_SIZE_BYTES);
          DF("in: %u out: %u\n", buff_pos_in, buff_pos_out);
          */
          // 25 reset packets
          for (uint8_t c=0; c<25; c++) {
            send_packet(packets, 2);
          }
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
            send_packet(packets, 2); // TODO send reset packets in service mode
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
  }
}
