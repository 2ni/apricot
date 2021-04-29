#include <avr/interrupt.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

volatile uint8_t done = 0;

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

void send_bit(uint8_t length) {
  PORTA.OUTSET = PIN7_bm;
  TCA0.SINGLE.PER = length ? 580 : 1160;
  done = 0;
  while (!done);
  PORTA.OUTCLR = PIN7_bm;
  done = 0;
  while (!done);
}

void send_byte(uint8_t data) {
  for (uint8_t c=0; c<8; c++) {
    send_bit(data & (1<<(7-c)));
  }
}

void send_packet(uint8_t addr, uint8_t data) {
  uint8_t c = 0;
  // preamble
  while (c<12) {
    send_bit(1);
    c++;
  }
  send_bit(0);
  send_byte(addr);
  send_bit(0);
  send_byte(data);
  send_bit(0);
  send_byte(addr ^ data);
  send_bit(1);
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
void track_switch(uint16_t addr, uint8_t local_addr, uint8_t output) {
  uint8_t a = 0x80 + (addr & 0x3f);
  //                 ----------- AAA -----------   - C -    ---------- BB ----------   ------ R ------
  uint8_t d = 0x80 + (((addr >> 6) ^ 0x07) << 4) + (1<<3) + ((local_addr & 0x03)<<1) + (output & 0x00);
  send_packet(a, d);
}

int main(void) {
  mcu_init();
  PORTA.DIRSET = PIN7_bm;

  timer_init();

  track_switch(0x01, 2, 1);
  send_bit(0);
  while(1);
  send_packet(0xff, 0x00); // idle packet

  uint8_t data = 0xf1;

  for (uint8_t c=0; c<8; c++) {
      PORTA.OUTSET = PIN7_bm;
      TCA0.SINGLE.PER = data & (1<<(7-c)) ? 580 : 1160;
      done = 0;
      while (!done);
      PORTA.OUTCLR = PIN7_bm;
      done = 0;
      while (!done);
  }
}
