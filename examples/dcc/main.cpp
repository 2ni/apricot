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
volatile uint8_t port = 0;
char buff[8];
volatile uint8_t buff_p = 0;
namespace CMD_STATE {
  typedef enum {
    NONE = 0,
    ADDR = 1,
  } Type_Cmd_State;
}
CMD_STATE::Type_Cmd_State cmd_state = CMD_STATE::NONE;

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
    uart_rollbefore(&pos, 8);
    data[num_of_chars-1-i] = buff[pos];
  }
  data[num_of_chars] = '\0';

  return num_of_chars;
}

/*
 * isr for uart read
 */
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  if (in != '\n') {
    buff[buff_p] = in;
    uart_rollover(&buff_p, 8);
  }

  if (cmd_state == CMD_STATE::NONE) {
    switch (in) {
      case '1': port = 1; break;
      case '2': port = 2; break;
      case '3': port = 3; break;
      case '4': port = 4; break;
      case '5': port = 5; break;
      case '6': port = 6; break;
      case '7': port = 7; break;
      case '8': port = 8; break;
      case '9': port = 9; break;
    }
  }

  if (in == '\n') {
    switch (cmd_state) {
      case CMD_STATE::NONE:
        char cmd[5];
        get_prev_chars(buff, cmd, 4);
        if (!strcmp(cmd, "addr")) {
          cmd_state = CMD_STATE::ADDR;
          D("enter new address...");
        }
        break;
      case CMD_STATE::ADDR:
        char addr_in[8];
        uint8_t l = get_prev_chars(buff, addr_in, 7);
        uint16_t addr = 0;
        for (uint8_t i=0; i<l; i++) {
          addr = addr*10 + addr_in[i] - '0';
        }
        eeprom_update_word(&ee_decoder_addr, addr);
        decoder_addr = addr;
        DF("address in use: %u\n", decoder_addr);
        cmd_state = CMD_STATE::NONE;
        break;
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
void send_packet(uint8_t *packets, uint8_t len) {
  // preamble
  for (uint8_t c=0; c<12; c++) send_bit(1);

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
 * Basic accessory decoder 11bit address
 * {preamble} 0 10AAAAAA 0 1AAACDDR 0 EEEEEEEE 1
 * example  weiche / turnout / track switch
 *
 * weiche 5 = module addr (AAAAAAAAA): 2, port (DD): 1 (port 1-4 are sent as 0-3)
 *            0 10000010 0 11110001 (0x82 0xf0)
 *
 * each module_addr (9bit) has 4 ports (1-4)
 */
void basic_accessory(uint16_t addr, uint8_t activation, uint8_t output) {
  uint8_t packets[2];
  // see https://wiki.rocrail.net/doku.php?id=addressing:accessory-pg-de
  // addr starts at 1
  uint8_t module_addr = (addr - 1) / 4 + 1;
  uint8_t port = (addr - 1) % 4 + 1;

  packets[0] = 0x80 | (module_addr & 0x3f);
  packets[1] = 0x80 | ((~module_addr & 0x1c0)>>2) | (((port - 1) & 0x03)<<1) | (activation ? 0x01<<3 : 0) | (output ? 0x01 : 0);
  pins_set(&PA7, 0);
  send_packet(packets, 2);
  pins_set(&PA7, 1);
}

/*
 * Basic accessory decoder 11bit address according to https://normen.railcommunity.de/RCN-213.pdf
 * {preamble} 0 10AAAAAA 0 1AADAAR 0 EEEEEEEE 1
 * D: activation
 * R: output (0=left, 1=right)
 *
 */
void basic_accessory11(uint16_t addr, uint8_t activation, uint8_t output) {
  uint8_t packets[2];
  packets[0] = 0x80 | ((addr & 0xfc)>>2);
  packets[1] = 0x80 | ((~addr & 0x700)>>4) | ((addr & 0x03)<<1) | (activation ? 0x01<<3 : 0) | (output ? 0x01 : 0);
  pins_set(&PA7, 0);
  send_packet(packets, 2);
  pins_set(&PA7, 1);
}

/*
 * Extended accessory decoder 11bit address
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
 * basic accessory decoder 9bit address configuration variable access
 * {preamble}   10AAAAAA 0 1AAACDDD 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 * example #P 0 10001011 0 10111000 0 11101100 0 00000000 0 00011000 0 11000111 1
 *
 * CC: 00=reserved, 01=verify byte, 11=writy byte, 10=bit manipulation
 * VVVVVVVVVV: 10bit address for CV1-1024
 * DDDDDDDD: data of CV
 * CDDD=0000 then the CVs refer to the entire decoder
 *
 * only write for now
 * cv_addr: 1-1024
 */
void basic_accessory_programming(uint16_t addr, uint16_t cv_addr, uint8_t cv_data) {
  uint8_t packets[5] = {0};
  cv_addr -= 1;
  packets[0] = 0x80 | (addr & 0x3f);
  packets[1] = 0x88 | ((~addr & 0x1c0)>>2);
  packets[2] = 0xe0 | (3<<2) | ((cv_addr & 0x300)>>8); // CC=11 (write byte)
  packets[3] = 0xff & cv_addr;
  packets[4] = cv_data;
  pins_set(&PA7, 0);
  send_packet(packets, 5);
  pins_set(&PA7, 1);
}

/*
 * Extended address mode multifunction decoder
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

void print_port(uint8_t port_outputs) {
  char buf[9];
  bitwise(buf, port_outputs);
  buf[8] = '\0';
  DF("ports (%u): %s\n", port, buf);
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

  timer_init();

  DL("usage:\n\
  1/2 : basic accessory 9bit\n\
  3/4 : multifunction decoder (extended address mode 14bit)\n\
  5   : one idle packet\n\
  6   : idle packets in between\n\
  7   : 25 reset packets\n\
  8   : send address change in ops mode\n\
  addr: change command address");

  while  (1) {
    switch (port) {
      case 1:
        basic_accessory(decoder_addr, 0, 0); // addr, activation, output
        // basic_accessory11(decoder_addr, 0, 1); // activation, output
        // extended_accessory(decoder_addr, 0);
        port_outputs &= !(1 << 7); // clear bit 7
        if (!fill_with_idle) print_port(port_outputs); // avoid times without packets
        port = 0;
        break;
      case 2:
        basic_accessory(decoder_addr, 0, 1);
        // basic_accessory11(decoder_addr, 1, 1);
        // extended_accessory(decoder_addr, 1);
        port_outputs |= (1 << 7); // set bit 7
        if (!fill_with_idle) print_port(port_outputs);
        port = 0;
        break;
      case 3:
        multifunction(decoder_addr, 0x1, 0);
        port_outputs &= ~(1<<3); // set bit 3
        if (!fill_with_idle) print_port(port_outputs);
        port = 0;
        break;
      case 4:
        multifunction(decoder_addr, 0x1, 1);
        port_outputs |= (1<<3); // set bit 3
        if (!fill_with_idle) print_port(port_outputs);
        port = 0;
        break;
      case 5:
        idle_packet();
        port = 0;
        break;
      case 6:
        fill_with_idle = !fill_with_idle;
        DF("fill with idle: %s\n", fill_with_idle ? "yes" : "no");
        port = 0;
        break;
      case 7:
        {
          // 25 reset packets
          uint8_t packets[2] = {0};
          pins_set(&PA7, 0);
          for (uint8_t c=0; c<25; c++) {
            send_packet(packets, 2);
          }
          pins_set(&PA7, 1);
          port = 0;
        }
        break;
      case 8:
        {
          // update cv1 and cv9 in ops mode
          // eg address 267 = 0x10b
          uint16_t new_addr = 24;
          basic_accessory_programming(decoder_addr, 1, new_addr & 0xff);        // CV1 = decoder address LSB
          // basic_accessory_programming(decoder_addr, 9, (new_addr >> 8) & 0x01); // CV9 = decoder address MSB
          if (!fill_with_idle) DF("service mode new addr: %u\n", new_addr);
          port = 0;
        }
        break;
      case 9:
        break;
    }

    if (fill_with_idle) {
      idle_packet();
    }
  }
}
