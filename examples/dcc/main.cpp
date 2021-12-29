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
volatile uint8_t trackpos = 0;

/*
 * isr for uart read
 */
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  switch (in) {
    case '1':
      trackpos = 1;
      break;
    case '2':
      trackpos = 2;
      break;
    case '\n':
      pins_flash(&pins_led, 1, 100);
      break;
  }
}

ISR(TCA0_OVF_vect) {
  done = 1;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  // PORTA.OUTTGL = PIN7_bm;
}


void timer_init() {
  TCA0.SINGLE.PER = 580; // 10MHz*58us = 580;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
}

void send_bit(uint8_t length, pins_t *pin) {
  pins_set(pin, 1);
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.PER = length ? 580 : 1160;
  done = 0;
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
 * example track (aka Weiche):
 * addr track: 10AAAAAA -> 0x80 + (realaddr & 0x3F)
 * data track: 1AAA1BBR -> 0x80 + (realaddr / 0x40) ^ 0x07) * 0x10
 *             A8-A6: are inverted
 *             BB   : local addr on the decoder (0, 1, 2, 3)
 *             R    : output bit
 *
 * we only support 3bytes (addr, data, xor)
 */
void send_packet(uint8_t addr, uint8_t data, pins_t *pin) {
  uint8_t c = 0;
  // preamble
  while (c<12) {
    send_bit(1, pin);
    c++;
  }
  send_bit(0, pin);
  send_byte(addr, pin);
  send_bit(0, pin);
  send_byte(data, pin);
  send_bit(0, pin);
  send_byte(addr ^ data, pin);
  send_bit(1, pin);
  // DF("a: 0x%02x, d: 0x%02x, c: 0x%02x\n", addr, data, addr^data);
}


/* example  weiche
 * preamble 12x 1bit
 * startbit 1x  0bit
 * addressbyte 10AAAAAA [A5...A0]
 * startbit 1x  0bit
 * databyte 1AAA1BBR [A8..A6](inverted), BB: local addr in decoder, R: output (1AAACDDD)
 * startbit 1x  0bit
 * cheksum address ^ data
 * stopbit  1x  1bit (or start of next preamble)
 *
 * addr: 8byte address
 * data: BBR;
 *
 * a: 0x95, d: 0xe2, c: 0x77
 * a: 0x81, d: 0xf2, c: 0x73 -> 000000001
 *
 * C: set output
 * BB: which port
 * R: which couple of the port (ports usually come in couples)
 */
void track_switch(uint16_t addr, uint8_t local_addr, uint8_t output, pins_t *pin) {
  uint8_t a = 0x80 + (addr & 0x3f); // a 128-191: 10AAAAAA, 1AAA1BBR (sometimes: 11AAAAAA, 1AAACDDD)
  //                 ----------- AAA -----------   - C -    ---------- BB ----------   ------ R ------
  uint8_t d = 0x80 + ((~addr & 0x1c0)>>2) + (1<<3) + ((local_addr & 0x03)<<1) + (output & 0x01);
  // DF("addr: 0x%03x, local: 0x%02x, a: 0x%02x, d: 0x%02x\n", addr, local_addr, a, d);
  send_packet(a, d, pin); // in  oscilloscope 9-bit-address is shown as msb!
  // control (see https://github.com/mrrwa/NmraDcc/blob/master/NmraDcc.cpp +1339)
  // int16_t ba = (((~d) & 0b01110000)<<2) | (a & 0b00111111);
  // DF("ba: %i\n", ba);
}

int main(void) {
  mcu_init(1);

  pins_t L = PB7;
  pins_t R = PB6;
  pins_t SIGNAL = PB6; // control signal

  pins_output(&L, 1);
  pins_output(&R, 1);
  pins_output(&SIGNAL, 1);

  timer_init();

  DL("usage:\n 1 = track position 0\n 2 = track position 1");

  while  (1) {
    // 1 or 2 from keyboard
    if (trackpos) {
      track_switch(0x003, 1, trackpos-1, &SIGNAL); // trackpos is 1(=off) or 2(=on)
      DF("trackpos: %u sent\n", trackpos-1);
      trackpos = 0;
    } else {
      send_packet(0xff, 0x00, &SIGNAL); // idle packet: preamble 0 11111111 0 00000000 0 11111111 1
      DL("idle packet");
    }
    send_bit(0, &SIGNAL);
    _delay_ms(2000);
  }
}
