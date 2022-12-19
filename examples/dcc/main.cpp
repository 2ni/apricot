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
char uart_cmd[6];
volatile uint8_t buff_p = 0;
volatile uint16_t pause_signal_at = 65535;
namespace CMD_STATE {
  typedef enum {
    NONE,
    TURN_LEFT,
    TURN_RIGHT,
    TOGGLE_IDLE,
    TOGGLE_EXTENDED,
    TOGGLE_PRG_MODE,
    RESET,
    PROCESS_UART_CMD,
    PROCESS_UART_DATA_WAITFOR,
    PROCESS_UART_DATA
  } Type_Cmd_State;
}
volatile CMD_STATE::Type_Cmd_State cmd_state = CMD_STATE::NONE;
volatile uint8_t to_print = 0;

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

// predefine buffer for idle packets
#define BUFF_IDLE_SIZE_BYTES 5
uint8_t buffer_idle_packets[BUFF_IDLE_SIZE_BYTES] = { 0xff, 0xf7, 0xf8, 0x01, 0xff };
uint8_t fill_with_idle = 0;
volatile int8_t idle_packets_p = -1;

// R = PB6, L = PB7 to control motor driver
volatile uint8_t output = 0;
uint16_t EEMEM ee_decoder_addr;
volatile uint16_t decoder_addr;

// we create a bit buffer of uint8_t's with size: BUFF_SIZE_BYTES * 8
// longest packet to be sent: prg new address with 2 bytes = 1834 bits ≈ 230bytes
#define BUFF_SIZE_BYTES 5 // min 2
uint8_t buffer[BUFF_SIZE_BYTES] = {0};
uint16_t buff_pos_in = 0;
volatile uint16_t buff_pos_out = 0; // max 20*8
volatile uint8_t current_bit = 0;
volatile uint8_t edgecount = 0;

uint8_t pause_buffer_processing() {
  uint8_t already_pausing = pause_signal_at != 65535;
  if (!already_pausing) pause_signal_at = buff_pos_in;

  return already_pausing;
}

void resume_buffer_processing(uint8_t already_pausing = 0) {
  if (!already_pausing) pause_signal_at = 65535;
}

/*
 * get current bit buffer to send out
 * (pointer buff_pos_out)
 */
uint8_t buff_get_current_bit(uint8_t *b, uint16_t p) {
  return (b[p/8] & (1<<(7-(p%8)))) ? 1 : 0;
}

void rollover(uint16_t *value, uint16_t max) {
*value = ++*value >= max ? 0 : *value;
}

void rollover(volatile uint16_t *value, uint16_t max) {
*value = ++*value >= max ? 0 : *value;
}

void rollover(volatile int8_t *value, uint8_t max) {
*value = ++*value >= max ? 0 : *value;
}

/*
 * add bit to transmission buffer
 * ~40us
 * (buff_add_bit)
 */
void send_bit(uint8_t bit) {
  uint16_t next = buff_pos_in;
  rollover(&next, BUFF_SIZE_BYTES*8);

  // wait for free space
  while (next == buff_pos_out);
  /*
  uint8_t warned = 0;
  while (next == buff_pos_out) {
    if (!warned) {
      warned = 1;
      DL("buf ofv");
    }
  }
  */

  uint8_t v = 1<<(7-(buff_pos_in%8));
  if (bit) {
    buffer[buff_pos_in/8] |= v;
  } else {
    buffer[buff_pos_in/8] &= ~v;
  }
  buff_pos_in = next;
}

/*
 * add byte to transmission buffer
 * ~200us
 * (buff_add_byte)
 */
void send_byte(uint8_t value) {
  for (uint8_t b=0; b<8; b++) {
    send_bit((value & (1<<(7-b))) ? 1 : 0);
  }
}

/*
 * interrupt set to 58us
 * handle 1=58us and 0=116us bit signals from our buffer
 */
// volatile uint8_t bitout = 0;
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;

  // no data to send and idle packet not done or not in action (idle_packets_p == 0 || idle_packets_p == -1)
  if (idle_packets_p <= 0 && !fill_with_idle && (buff_pos_out == buff_pos_in || (pause_signal_at != 65535 && buff_pos_out == pause_signal_at))) {
    PORTC.OUT |= PIN4_bm | PIN5_bm; // PB6=PB7=1
    edgecount = 0;
    return;
  }

  // let's toggle as fast as possible in isr
  if (edgecount == 0 || current_bit == 1 || (current_bit == 0 && (edgecount == 2 || edgecount == 4))) {
    uint8_t portvalues = PORTC.IN;
    // clear bit 7, copy pin 6 to 7, then toggle pin 6
    portvalues = (portvalues & ~PIN5_bm) | ((portvalues & PIN4_bm)>>PIN4_bp<<PIN5_bp);
    portvalues ^= PIN4_bm;
    PORTC.OUT = portvalues;
  }

  // signal start
  if (edgecount == 0) {
    uint8_t has_data = (buff_pos_in != buff_pos_out) && (pause_signal_at == 65535 || buff_pos_out != pause_signal_at);
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
      current_bit = buff_get_current_bit(buffer_idle_packets, idle_packets_p);
    }
    /*
    DF("%u", current_bit);
    if (bitout++%8 == 7) D(" ");
    */
  }

  edgecount++;

  // signal done
  if ((current_bit == 1 && edgecount == 2) || (current_bit == 0 && edgecount == 4)) {
    // get next bit within it buffer is 0 1 2 3 4 5 6 7 8 9...
    //                                 |    buff[0]    | buff[1]...
    if (idle_packets_p == -1) {
      rollover(&buff_pos_out, BUFF_SIZE_BYTES*8);
    }
    else {
      rollover(&idle_packets_p, BUFF_IDLE_SIZE_BYTES*8);
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
    value = value*base + (data_in[i] >= 'a' ? data_in[i] - 'a' + 10 : data_in[i] - '0');
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
      case '0'+CMD_STATE::TOGGLE_PRG_MODE: cmd_state = CMD_STATE::TOGGLE_PRG_MODE; break;
      case '0'+CMD_STATE::RESET: cmd_state = CMD_STATE::RESET; break;
    }
  }

  if (in == '\n') {
    cmd_state = cmd_state == CMD_STATE::PROCESS_UART_DATA_WAITFOR ? CMD_STATE::PROCESS_UART_DATA : CMD_STATE::PROCESS_UART_CMD;
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
  // uint8_t pause = pause_buffer_processing();

  // preamble (ops: 12bits, service: 20bits)
  send_byte(0xff);
  if (mode == PRG::SERVICE) send_byte(0xff);
  for (uint8_t c=0; c<4; c++) send_bit(1);

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

  // resume_buffer_processing(pause);

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
  *port = (addr - 1) % 4;
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
 *
 * basic (decoder addr)  9bit: {preamble} 0 10AAAAAA 0 1aaaCDDR 0 EEEEEEEE 1
 * basic (output addr)  11bit: {preamble} 0 10AAAAAA 0 1aaaCAAR 0 EEEEEEEE 1
 * extended 11bit:             {preamble} 0 10AAAAAA 0 0aaa0AA1 0 DDDDDDDD 0 EEEEEEEE 1
 *                                          10A7A6A5A4A3A2 0 0a10a9a80A1A01 0
 *
 * A: address bit
 * a: address bit complement
 * C: power/activation (0=inactive, 1=active)
 * D: port (0-3)
 * R: direction/output (0:left/diverting/stop, 1:right/straight/run)
 *
 */
void basic_accessory(uint16_t addr, uint8_t activation, uint8_t output, uint8_t is_roco = 0) {
  uint8_t packets[2];
  uint8_t module_addr;
  uint8_t port;
  split_addr(addr, &module_addr, &port, is_roco);

  // we use output addressing from the given address
  // the decoder can interpret the incoming data as output or decoder addressing
  packets[0] = 0x80 | (module_addr & 0x3f);
  packets[1] = 0x80 | ((~module_addr & 0x1c0)>>2) | ((port & 0x03)<<1) | (activation ? 0x01<<3 : 0) | (output ? 0x01 : 0);
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
 *
 * longest packet:
 *   service: 25 reset packets + 1 data packet with 4 bytes = 25*(12+3*8+4) + 20+4*8+5 = 1057 ≈ 133bytes
 *   ops: 2 data packets with 6 bytes = 2*(12+6*8+7) = 134bits ≈ 17bytes
 */
void basic_accessory_prg(uint16_t addr, PRG::Type_Prg_Mode mode, PRG::Type_Prg_Cmd_Type cmd_type, uint16_t cv_addr, uint8_t cv_data, uint8_t skip_resets = 0, uint8_t is_roco = 0) {
  uint8_t packets[5] = {0};
  uint8_t module_addr;
  uint8_t port;
  uint8_t index = 0;
  uint8_t num = 3;
  uint8_t cmd_start = 0b0111<<4; // service

  // uint8_t pause = pause_buffer_processing();

  // send reset packets 1st (only in service mode before 1st packet)
  if (mode == PRG::SERVICE && !skip_resets) {
    // changed: we only send 1 reset packet instead of 25
    for (uint8_t c=0; c<1; c++) {
      send_packet(packets, 2, mode);
    }
  }

  // add address packets
  split_addr(addr, &module_addr, &port, is_roco);
  if (mode == PRG::OPS) {
    packets[0] = 0x80 | (module_addr & 0x3f);
    packets[1] = 0x88 | ((~module_addr & 0x1c0)>>2) | ((port & 0x03)<<1);
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
  if (mode == PRG::OPS && (cmd_type == PRG::WRITE || (cmd_type == PRG::BIT && cv_data & 0x10))) send_packet(packets, num, mode);

  // uart_arr("prg", packets, num);
  // activate (=send) newest buffer entries
  // resume_buffer_processing(pause);
}

void basic_accessory_prg_bit(uint16_t addr, PRG::Type_Prg_Mode mode, uint16_t cv_addr, uint8_t write, uint8_t bit_addr, uint8_t bit_value, uint8_t is_roco = 0) {
  uint8_t cv_data = 0xe0 | (write ? 0x10 : 0x00) | (bit_value ? 0x08 : 0x00) | (0x07 & bit_addr);
  basic_accessory_prg(addr, mode, PRG::BIT, cv_addr, cv_data, 0, is_roco);
}

/*
 * extended accessory decoder 11bit address
 * uses output addressing as in basic accessory
 * {preamble} 0 10AAAAAA 0 0aaa0AA1 0 000XXXXX 0 EEEEEEEE 1
 *              10A7A6A5A4A3A2 0 0a10a9a80A1A01 0
 */
void extended_accessory(uint16_t addr, uint8_t output) {
  basic_accessory(addr, 0, output);
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
void extended_accessory_prg(uint16_t addr, PRG::Type_Prg_Mode mode, PRG::Type_Prg_Cmd_Type cmd_type, uint8_t cv_addr, uint8_t cv_data, uint8_t skip_resets = 0) {
  basic_accessory_prg(addr, mode, cmd_type, cv_addr, cv_data, skip_resets);
}

void extended_accessory_prg_bit(uint16_t addr, PRG::Type_Prg_Mode mode, uint16_t cv_addr, uint8_t write, uint8_t bit_addr, uint8_t bit_value) {
  uint8_t cv_data = 0xe0 | (write ? 0x10 : 0x00) | (bit_value ? 0x08 : 0x00) | (0x07 & bit_addr);
  extended_accessory_prg(addr, mode, PRG::BIT, cv_addr, cv_data);
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

  PORTC.DIR |= PIN4_bm | PIN5_bm;
  PORTC.OUT |= PIN4_bm | PIN5_bm; // PB6=PB7=1

  // debug pin for oscilloscope
  pins_output(&PA7, 1);
  pins_set(&PA7, 1);

  uint8_t port_outputs = 0;
  uint8_t is_extended_mode = 0;
  PRG::Type_Prg_Mode prg_mode = PRG::SERVICE;

  // show help on start
  cmd_state = CMD_STATE::PROCESS_UART_CMD;
  strncpy(buff, "help", 4);
  buff_p = 4;

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
        print_outputs(port_outputs);
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TURN_RIGHT:
        if (is_extended_mode) {
          extended_accessory(decoder_addr, 1);
        } else {
          basic_accessory(decoder_addr, 0, 1, 0);
        }
        port_outputs |= (1 << 7); // set bit 7
        print_outputs(port_outputs);
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_IDLE:
        fill_with_idle = !fill_with_idle;
        DF("(%u) fill with idle: %s\n", CMD_STATE::TOGGLE_IDLE, fill_with_idle ? "yes" : "no");
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_EXTENDED:
        is_extended_mode = !is_extended_mode;
        DF("(%u) mode: %s\n", CMD_STATE::TOGGLE_EXTENDED, is_extended_mode ? "extended" : "basic");
        cmd_state = CMD_STATE::NONE;
        break;
      case CMD_STATE::TOGGLE_PRG_MODE:
        prg_mode = prg_mode == PRG::OPS ? PRG::SERVICE : PRG::OPS;
        DF("(%u) prg mode: %s\n", CMD_STATE::TOGGLE_PRG_MODE, prg_mode == PRG::OPS ? "ops" : "service");
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
          DL("25 reset packets sent");
          cmd_state = CMD_STATE::NONE;
        }
        break;
      case CMD_STATE::PROCESS_UART_CMD:
        if (get_prev_chars(buff, uart_cmd, 4) && !strcmp(uart_cmd, "help")) {
          DL("usage:");
          DF("  %u : turn left\n", CMD_STATE::TURN_LEFT);
          DF("  %u : turn right\n", CMD_STATE::TURN_RIGHT);
          DF("  %u : %s|%s idle packets in between (toggle)\n", CMD_STATE::TOGGLE_IDLE, fill_with_idle ? OK("yes") : "yes", fill_with_idle ? "no": OK("no"));
          DF("  %u : %s|%s mode (toggle)\n", CMD_STATE::TOGGLE_EXTENDED, is_extended_mode ? "basic" : OK("basic"), is_extended_mode ? OK("extended") : "extended");
          DF("  %u : %s|%s prg mode (toggle)\n", CMD_STATE::TOGGLE_PRG_MODE, prg_mode == PRG::OPS ? "service" : OK("service"), prg_mode == PRG::OPS ? OK("ops") : "ops");
          DF("  %u : 25 reset packets\n", CMD_STATE::RESET);
          DF("  saddr: " OK("%u/0x%03x") " address cmds are sent to\n", decoder_addr, decoder_addr);
          DL("  caddr: change device addr");
          DL("  cv  : verify/write cv");
          DL("  cvb : verify/write cv bit");
          DL("  help: show this help");
          cmd_state = CMD_STATE::NONE;
        }
        else if (get_prev_chars(buff, uart_cmd, 5) && !strcmp(uart_cmd, "caddr")) {
          D("new addr for device: ");
          cmd_state = CMD_STATE::PROCESS_UART_DATA_WAITFOR;
        }
        else if (get_prev_chars(buff, uart_cmd, 5) && !strcmp(uart_cmd, "saddr")) {
          D("new addr cmds are sent to: ");
          cmd_state = CMD_STATE::PROCESS_UART_DATA_WAITFOR;
        }
        else if (get_prev_chars(buff, uart_cmd, 2) && !strcmp(uart_cmd, "cv")) {
          D("<cv addr>[r|w]<cv value> (eg 121r0x05): ");
          cmd_state = CMD_STATE::PROCESS_UART_DATA_WAITFOR;
        }
        else if (get_prev_chars(buff, uart_cmd, 3) && !strcmp(uart_cmd, "cvb")) {
          D("<cv addr>[r|w]<bitpos><bitvalue> (eg 121r71): ");
          cmd_state = CMD_STATE::PROCESS_UART_DATA_WAITFOR;
        }
        else {
          cmd_state = CMD_STATE::NONE;
        }
        // clear ring buffer
        buff_p = 0;
        for (uint8_t ii=0; ii<20; ii++) buff[ii] = 0;
        break;
      case CMD_STATE::PROCESS_UART_DATA:
        get_prev_chars(buff, uart_data, 20);
        // process cmd with data
        if (!strcmp(uart_cmd, "caddr")) {
          uint16_t addr = char2int(uart_data);
          // lsb
          if (is_extended_mode) {
            extended_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 121, addr & 0xff);
          } else {
            basic_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 121, addr & 0xff);
          }
          if (prg_mode == PRG::SERVICE) {
            // 1 reset 7.772ms -> 14 packets
            // wait 100ms filling with reset (=12.86670098 reset packets) as we can't read ack from decoder yet
            uint8_t packets[2] = {0};
            for (uint8_t i=0; i<13; i++) {
              send_packet(packets, 2);
            }
          }
          // msb (skip intro reset packets in service mode, none in ops mode anyways)
          if (is_extended_mode) {
            extended_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 120, (addr >> 8) & 0xff);
          } else {
            basic_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 120, (addr >> 8) & 0xff, 1);
          }
          DF("\nnew addr: %u/0x%03x\n", addr, addr);
        }
        else if (!strcmp(uart_cmd, "saddr")) {
          uint16_t addr = buff2int(uart_data);
          eeprom_update_word(&ee_decoder_addr, addr);
          decoder_addr = addr;
          DF("address in use: %u (0x%04x)\n", decoder_addr, decoder_addr);
        }
        else if (!strcmp(uart_cmd, "cv")) {
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
          if (is_extended_mode) {
            extended_accessory_prg(decoder_addr, prg_mode, cmd_type, cv_addr, cv_value);
          } else {
            basic_accessory_prg(decoder_addr, prg_mode, cmd_type, cv_addr, cv_value);
          }
        }
        else if (!strcmp(uart_cmd, "cvb")) {
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
          if (is_extended_mode) {
            extended_accessory_prg_bit(decoder_addr, prg_mode, cv_addr, write, bit_addr, bit_value);
          } else {
            basic_accessory_prg_bit(decoder_addr, prg_mode, cv_addr, write, bit_addr, bit_value);
          }
        }

        uart_data[0] = '\0';
        // clear ring buffer
        buff_p = 0;
        for (uint8_t ii=0; ii<20; ii++) buff[ii] = 0;
        cmd_state = CMD_STATE::NONE;
        break;
      default:
        break;
    }
  }
}
