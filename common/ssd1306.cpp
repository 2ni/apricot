#include <string.h>

#include "ssd1306.h"
#include "twi.h"
#include "uart.h"

SSD1306::SSD1306() {
  SSD1306::addr = 0x3c;
}

SSD1306::SSD1306(uint8_t addr) {
  SSD1306::addr = addr;
}

// TODO return error code
void SSD1306::data() {
  if (twi_start(SSD1306::addr) != 0) DF(NOK("err start i2c data 0x%02x") "\n", SSD1306::addr);
  twi_write(0x40);
}

void SSD1306::cmd() {
  if (twi_start(SSD1306::addr) != 0) DF(NOK("err start i2c cmd 0x%02x") "\n", SSD1306::addr);
  twi_start(SSD1306::addr);
  twi_write(0);
}

void SSD1306::cmd(uint8_t cmd) {
  SSD1306::cmd();
  twi_write(cmd);
  twi_stop();
}

// return error code and print if error
void SSD1306::init() {
  twi_init();
  for (uint8_t i=0; i<sizeof(SSD1306::init_cmds); i++) {
    SSD1306::cmd();
    twi_write(SSD1306::init_cmds[i]);
    twi_stop();
    SSD1306::data();
    twi_stop();
  }
  SSD1306::clear();
}

void SSD1306::init_transfer () {
  for (uint8_t i=0; i<sizeof(SSD1306::init_transfer_cmds); i++) {
    SSD1306::cmd();
    twi_write(SSD1306::init_transfer_cmds[i]);
    twi_stop();
  }
}

void SSD1306::off() {
  SSD1306::cmd(0xae);
}

void SSD1306::on() {
  SSD1306::cmd(0xaf);
}

void SSD1306::clear() {  //nn<128x8=1024
  SSD1306::init_transfer();
  SSD1306::data();
  for (uint16_t i=0; i<1024; i++) {
    twi_write(0x00);
  }
  twi_stop();
}

/*
 * row: 0-7
 * col: 0-124 (or 123 depending on char size)
 */
void SSD1306::clear(uint8_t row, uint8_t col, uint8_t width) {
  SSD1306::set_pos(row, col);
  SSD1306::data();
  for (uint8_t i=0; i<width; i++) {
    twi_write(0x00);
  }
  twi_stop();
}

void SSD1306::set_row(int8_t row) {
  SSD1306::cmd();
  twi_write(0x22);
  twi_write(row);
  twi_write(7);
  twi_stop();
}

void SSD1306::set_col(uint8_t col) {
  SSD1306::cmd();
  twi_write(0x21);
  twi_write(col);
  twi_write(127);
  twi_stop();
}

/*
 * row: 0-7
 * col: 0-124 (or 123 depending on char size)
 */
void SSD1306::set_pos(uint8_t row, uint8_t col) {
  SSD1306::set_row(row);
  SSD1306::set_col(col);
}

// y: 0-64 -> 0-7
void SSD1306::dot(uint8_t x, uint8_t y, uint8_t width) {
  SSD1306::set_pos(y/8, x);
  SSD1306::data();
  twi_write(((1<<width)-1) << (y%8)); // deletes all other bits
  twi_stop();
}

/*
 * horizontal line on position y: 0-64
 * width: width starting from x=0
 * height: 1-8px
 */
void SSD1306::hline(uint8_t y, uint8_t from, uint8_t width, uint8_t height) {
  for (uint8_t i=from; i<(from+width); i++) {
    SSD1306::dot(i, y, height);
  }
}

void SSD1306::vline (uint8_t x) {
  for (uint8_t i=0; i<8; i++) {
    SSD1306::set_pos(i, x);
    SSD1306::data ();
    twi_write(0xFF);
    twi_stop();
  }
}

void SSD1306::normalchar(char c, uint8_t row, uint8_t col) {
  SSD1306::set_pos(row, col);
  SSD1306::data();
  for (uint8_t i=0; i<5; i++) {
    twi_write(SSD1306::font[c-32][i]); // start at pos 32 ascii table, 5 bytes
  }
  twi_write(0);
  twi_stop();
}

uint8_t SSD1306::pow(uint8_t value) {
  for (uint8_t i=7; i>=0; i--) {
    if ((1<<i) & value) return i;
  }
  return 0;
}

/*
 * scale: 1, 2, 4, 8
 */
void SSD1306::largechar(char c, uint8_t row, uint8_t col, uint8_t scale) {
  uint8_t d_char;
  uint8_t d_char_masked_window;
  uint8_t d_char_chunk;

  // window of bits to consider, eg 1111 for factor 2
  uint8_t mask = ((1<<(8>>SSD1306::pow(scale)))-1);
  uint8_t mask_len = 0;
  for (uint8_t i=0; i<8; i++) mask_len += (mask & (1<<i))>>i;

  for (uint8_t i=0; i<5; i++) {
    d_char = SSD1306::font[c-32][i];
    for (uint8_t h=0; h<scale; h++) { // scale vertically
      d_char_masked_window = (d_char & (mask<<(mask_len*h)))>>(mask_len*h); // get current window
      d_char_chunk = 0;
      for (uint8_t p=0; p<mask_len; p++) { // create vertical stretched byte
        if (d_char_masked_window & (1<<p)) {
          for (uint8_t s=0; s<scale; s++) {
            d_char_chunk |= (1<<(p*scale+s));
          }
        }
      }

      for (uint8_t w=0; w<scale; w++) { // scale horizontally
        SSD1306::set_pos(row+h, col+i*scale+w);
        SSD1306::data();
        twi_write(d_char_chunk);
        twi_write(0);
        twi_stop();
      }
    }
  }
}

uint8_t SSD1306::text(const char *text, uint8_t row, uint8_t col, uint8_t scale) {
  SSD1306::set_pos(row, col);
  for (uint8_t i=0; i<strlen(text); i++) {
    SSD1306::largechar(text[i], row, col, scale);
    col += scale*(5+1);
  }
  return col;
}
