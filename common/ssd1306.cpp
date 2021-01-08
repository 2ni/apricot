#include <avr/pgmspace.h>
#include <string.h>

#include "ssd1306.h"
#include "fonts.h"
#include "twi.h"
#include "uart.h"

// TODO return error code
void ssd1306_data() {
  if (twi_start(SSD1306_ADDR) != 0) DF(NOK("err start i2c data 0x%02x") "\n", SSD1306_ADDR);
  twi_write(0x40);
}

void ssd1306_cmd() {
  if (twi_start(SSD1306_ADDR) != 0) DF(NOK("err start i2c cmd 0x%02x") "\n", SSD1306_ADDR);
  twi_start(SSD1306_ADDR);
  twi_write(0);
}

void ssd1306_cmd(uint8_t cmd) {
  ssd1306_cmd();
  twi_write(cmd);
  twi_stop();
}

// return error code and print if error
void ssd1306_init() {
  twi_init();
  for (uint8_t i=0; i<sizeof(ssd1306_init_cmds); i++) {
    ssd1306_cmd();
    twi_write(ssd1306_init_cmds[i]);
    twi_stop();
    ssd1306_data();
    twi_stop();
  }
  ssd1306_clear();
}

void ssd1306_init_transfer () {
  for (uint8_t i=0; i<sizeof(ssd1306_init_transfer_cmds); i++) {
    ssd1306_cmd();
    twi_write(ssd1306_init_transfer_cmds[i]);
    twi_stop();
  }
}

void ssd1306_off() {
  ssd1306_cmd(0xae);
}

void ssd1306_on() {
  ssd1306_cmd(0xaf);
}

void ssd1306_clear() {  //nn<128x8=1024
  ssd1306_init_transfer();
  ssd1306_data();
  for (uint16_t i=0; i<1024; i++) {
    twi_write(0x00);
  }
  twi_stop();
}

/*
 * row: 0-7
 * col: 0-124 (or 123 depending on char size)
 */
void ssd1306_clear(uint8_t row, uint8_t col, uint8_t width) {
  ssd1306_set_pos(row, col);
  ssd1306_data();
  for (uint8_t i=0; i<width; i++) {
    twi_write(0x00);
  }
  twi_stop();
}

void ssd1306_set_row(int8_t row) {
  ssd1306_cmd();
  twi_write(0x22);
  twi_write(row);
  twi_write(7);
  twi_stop();
}

void ssd1306_set_col(uint8_t col) {
  ssd1306_cmd();
  twi_write(0x21);
  twi_write(col);
  twi_write(127);
  twi_stop();
}

/*
 * row: 0-7
 * col: 0-124 (or 123 depending on char size)
 */
void ssd1306_set_pos(uint8_t row, uint8_t col) {
  ssd1306_set_row(row);
  ssd1306_set_col(col);
}

// y: 0-64 -> 0-7
void ssd1306_dot(uint8_t x, uint8_t y, uint8_t width) {
  ssd1306_set_pos(y/8, x);
  ssd1306_data();
  twi_write(((1<<width)-1) << (y%8)); // deletes all other bits
  twi_stop();
}

/*
 * horizontal line on position y: 0-64
 * width: width starting from x=0
 * height: 1-8px
 */
void ssd1306_hline(uint8_t y, uint8_t from, uint8_t width, uint8_t height) {
  for (uint8_t i=from; i<(from+width); i++) {
    ssd1306_dot(i, y, height);
  }
}

void ssd1306_vline (uint8_t x) {
  for (uint8_t i=0; i<8; i++) {
    ssd1306_set_pos(i, x);
    ssd1306_data ();
    twi_write(0xFF);
    twi_stop();
  }
}

void ssd1306_char(char c, uint8_t row, uint8_t col) {
  ssd1306_set_pos(row, col);
  ssd1306_data();
  for (uint8_t i=0; i<5; i++) {
    twi_write(pgm_read_byte(&font[c-32][i])); // start at pos 32 ascii table, 5 bytes
  }
  twi_write(0);
  twi_stop();
}

uint8_t ssd1306_text(const char *text, uint8_t row, uint8_t col) {
  ssd1306_set_pos(row, col);
  for (uint8_t i=0; i<strlen(text); i++) {
    ssd1306_char(text[i], row, col);
    col += 5+1;
  }
  return col;
}
