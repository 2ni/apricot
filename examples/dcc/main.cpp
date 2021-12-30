/*
 * creates a DCC signal (simulator)
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"

volatile uint8_t done = 0;
volatile uint8_t port = 0;

/*
 * isr for uart read
 */
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  switch (in) {
    case '1': port = 1; break;
    case '2': port = 2; break;
    case '3': port = 3; break;
    case '4': port = 4; break;
    case '5': port = 5; break;
    case '6': port = 6; break;
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
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
}

void send_bit(uint8_t length, pins_t *pin) {
  pins_set(pin, 1);
  done = 0;
  TCA0.SINGLE.PER = length ? 580 : 1160; // 10MHz*58us = 580
  TCA0.SINGLE.CNT = 0;
  while (!done); // wait for interrupt
  pins_set(pin, 0);
  done = 0;
  while (!done); // wait for interrupt
}

void send_byte(uint8_t data, pins_t *pin) {
  for (uint8_t c=0; c<8; c++) {
    send_bit(data & (1<<(7-c)), pin);
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
void send_packet(uint8_t *packets, uint8_t len, pins_t *pin) {
  // preamble
  for (uint8_t c=0; c<12; c++) send_bit(1, pin);

  // data
  uint8_t x = 0;
  for (uint8_t c=0; c<len; c++) {
    send_bit(0, pin);
    send_byte(packets[c], pin);
    x ^= packets[c];
  }
  send_bit(0, pin);
  send_byte(x, pin);
  send_bit(1, pin);
  pins_set(pin, 1);
  // uart_arr("packets", packets, len);
  // DF("xor: 0x%02x\n", x);
  // DF("a: 0x%02x, d: 0x%02x, c: 0x%02x\n", addr, data, addr^data);
}

/*
 * Basic accessory devoder 9bit address
 * example  weiche / turnout / track switch
 *
 */
void track_switch(uint16_t addr, uint8_t local_addr, uint8_t output, pins_t *pin) {
  uint8_t packets[2];
  packets[0] = 0x80 + (addr & 0x3f); // a 128-191: 10AAAAAA, 1AAA1BBR (sometimes: 11AAAAAA, 1AAACDDD)
  //                  ----------- AAA -----------   - C -    ---------- BB ----------   ------ R ------
  packets[1] = 0x80 + ((~addr & 0x1c0)>>2) + (1<<3) + ((local_addr & 0x03)<<1) + (output & 0x01);
  // DF("addr: 0x%03x, local: 0x%02x, a: 0x%02x, d: 0x%02x\n", addr, local_addr, a, d);
  send_packet(packets, 2, pin);
}

/*
 * Extended address mode multifunction devoder
 *
 * type see https://dccwiki.com/Digital_Packet
 * b000 decoder and consist control instructions
 * b001 advanced operations
 * ...
 *
 * only supports one instruction byte (extended address mode 14bit address)
 * 0x0000 - 0x3fff (0xc00 - 0xfff)
 * {preamble} 0 11AAAAAA 0 AAAAAAAA CCCDDDDD 0 EEEEEEEE 1
 * A: address bit
 * C: type bit
 * D: data bit
 * E: control bit (xor)
 */
void multifunction_decoder(uint16_t addr, uint16_t type, uint8_t data, pins_t *pin) {
  uint8_t packets[3];
  packets[0] = (0xc0 | (addr >> 8)); // packet must be 0b11xxxxxx
  packets[1] = addr & 0xff;
  packets[2] = ((type & 0x07)<<5) | (data & 0x1f);
  send_packet(packets, 3, pin);
  uart_arr("multi", packets, 3);
}

void bitwise(char *buf, uint8_t value) {
  for (uint8_t i=0; i<8; i++) {
    buf[i] = value & 0x80 ? '1' : '0';
    value <<= 1;
  }
}

int main(void) {
  mcu_init(1);

  pins_t L = PB7;
  pins_t R = PB6;
  pins_t SIGNAL = PB6; // control signal

  pins_output(&L, 1);
  pins_output(&R, 1);
  pins_output(&SIGNAL, 1);
  pins_set(&SIGNAL, 1);

  uint8_t port_outputs = 0;
  char buf[9];

  timer_init();

  DL("usage:\n 1-4: basic accessory 9bit\n 5: multifunction decoder (extended address mode 14bit)\n 6: idle packet");

  while  (1) {
    switch (port) {
      case 1:
      case 2:
      case 3:
      case 4:
        track_switch(0x03, port-1, (port_outputs >> (port-1)) ? 0 : 1, &SIGNAL);
        port_outputs ^= 1 << (port-1); // toggle bit port-1
        bitwise(buf, port_outputs);
        buf[8] = '\0';
        DF("port %u: %s\n", port, buf);
        port = 0;
        break;
      case 5:
        multifunction_decoder(0x7d0, 0x1, (port_outputs >> (port-1)) ? 0 : 1, &SIGNAL);
        /*
        uint8_t packets[2];
        packets[0] = 0x05;
        packets[1] = 0x64;
        send_packet(packets, 2, &SIGNAL);
        */

        /*
        for (uint8_t i=0; i<12; i++) send_bit(1, &SIGNAL);
        send_bit(0, &SIGNAL);
        send_byte(0x05, &SIGNAL);
        send_bit(0, &SIGNAL);
        send_byte(0x64, &SIGNAL);
        send_bit(0, &SIGNAL);
        send_byte(0x61, &SIGNAL);
        send_bit(1, &SIGNAL);
        pins_set(&SIGNAL, 1);
        */

        /*
        uint8_t packets[2];
        packets[0] = 0x80 | 0x03;
        packets[1] = 0x00;
        send_packet(packets, 2, &SIGNAL);
        */

        port_outputs ^= 1 << (port-1); // toggle bit port-1
        bitwise(buf, port_outputs);
        buf[8] = '\0';
        DF("port %u: %s\n", port, buf);
        port = 0;
        break;
      case 6:
        uint8_t packets[2];
        packets[0] = 0xff;
        packets[1] = 0x00;
        send_packet(packets, 2, &SIGNAL); // idle packet
        port = 0;
        break;
    }
  }
}
