#ifndef OLED_h
#define OLED_h

#define SSD1306_ADDR 0x3C

#include <avr/pgmspace.h>

const uint8_t ssd1306_init_cmds[] = { 0xae, 0xd5, 0x80, 0xa8, 63, \
  0xD3, 0x0, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, \
  0xc8, 0xDA, 0x12, 0x81, 0xcf, 0xd9, 0xf1, 0xdb, \
  0x40, 0xa4, 0xa6, 0x2e, 0xaf };

const uint8_t ssd1306_init_transfer_cmds[] = { 0x21, 0x0, 0x7f, 0x22, 0x0, 0x7F };

void    ssd1306_clear();
void    ssd1306_clear(uint8_t row, uint8_t col, uint8_t width);
void    ssd1306_data();
void    ssd1306_cmd();
void    ssd1306_cmd (uint8_t cmd);
void    ssd1306_init();
void    ssd1306_init_transfer();
void    ssd1306_off();
void    ssd1306_on();
void    ssd1306_set_line(int8_t line);
void    ssd1306_set_col(uint8_t col);
void    ssd1306_set_pos(uint8_t line, uint8_t col);
void    ssd1306_dot(uint8_t xx, uint8_t yy, uint8_t width=1);
void    ssd1306_hline(uint8_t y, uint8_t from, uint8_t width, uint8_t height=1);
void    ssd1306_vline(uint8_t x);
void    ssd1306_char(char c, uint8_t row, uint8_t col);
uint8_t ssd1306_text(const char *text, uint8_t row, uint8_t col);

#endif
