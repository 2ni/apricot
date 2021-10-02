#ifndef __SSD1306_h__
#define __SSD1306_h__

#include "pins.h"

class SSD1306 {
  public:
    SSD1306(uint8_t addr);
    SSD1306();
    void    clear();
    void    clear(uint8_t row, uint8_t col, uint8_t width);
    void    set_row(int8_t row);
    void    data();
    void    cmd();
    void    cmd (uint8_t cmd);
    void    init();
    void    init_transfer();
    void    off();
    void    on();
    void    set_line(int8_t line);
    void    set_col(uint8_t col);
    void    set_pos(uint8_t line, uint8_t col);
    void    dot(uint8_t xx, uint8_t yy, uint8_t width=1);
    void    hline(uint8_t y, uint8_t from, uint8_t width, uint8_t height=1);
    void    vline(uint8_t x);
    void    normalchar(char c, uint8_t row, uint8_t col);
    void    largechar(char c, uint8_t row, uint8_t col, uint8_t scale=2);
    uint8_t text(const char *text, uint8_t row, uint8_t col, uint8_t scale=1);

  private:
    const uint8_t init_cmds[26] = { 0xae, 0xd5, 0x80, 0xa8, 63, \
      0xD3, 0x0, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, \
      0xc8, 0xDA, 0x12, 0x81, 0xcf, 0xd9, 0xf1, 0xdb, \
      0x40, 0xa4, 0xa6, 0x2e, 0xaf };

    const uint8_t init_transfer_cmds[6] = { 0x21, 0x0, 0x7f, 0x22, 0x0, 0x7F };
    uint8_t addr;

    uint8_t pow(uint8_t value);

    const uint8_t font[96][6] = {
      { 0x00, 0x00, 0x00, 0x00, 0x00 }, // space (32)
      { 0x00, 0x00, 0x5F, 0x00, 0x00 }, // !
      { 0x00, 0x07, 0x00, 0x07, 0x00 }, // "
      { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, // #
      { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, // $
      { 0x23, 0x13, 0x08, 0x64, 0x62 }, // %
      { 0x36, 0x49, 0x56, 0x20, 0x50 }, // &
      { 0x00, 0x08, 0x07, 0x03, 0x00 }, // '
      { 0x00, 0x1C, 0x22, 0x41, 0x00 }, // (
      { 0x00, 0x41, 0x22, 0x1C, 0x00 }, // )
      { 0x2A, 0x1C, 0x7F, 0x1C, 0x2A }, // *
      { 0x08, 0x08, 0x3E, 0x08, 0x08 }, // +
      { 0x00, 0x80, 0x70, 0x30, 0x00 }, // ,
      { 0x08, 0x08, 0x08, 0x08, 0x08 }, // -
      { 0x00, 0x00, 0x60, 0x60, 0x00 }, // .
      { 0x20, 0x10, 0x08, 0x04, 0x02 }, // /
      { 0x3E, 0x51, 0x49, 0x45, 0x3E }, // 0
      { 0x00, 0x42, 0x7F, 0x40, 0x00 }, // 1
      { 0x72, 0x49, 0x49, 0x49, 0x46 }, // 2
      { 0x21, 0x41, 0x49, 0x4D, 0x33 }, // 3
      { 0x18, 0x14, 0x12, 0x7F, 0x10 }, // 4
      { 0x27, 0x45, 0x45, 0x45, 0x39 }, // 5
      { 0x3C, 0x4A, 0x49, 0x49, 0x31 }, // 6
      { 0x41, 0x21, 0x11, 0x09, 0x07 }, // 7
      { 0x36, 0x49, 0x49, 0x49, 0x36 }, // 8
      { 0x46, 0x49, 0x49, 0x29, 0x1E }, // 9
      { 0x00, 0x00, 0x14, 0x00, 0x00 }, // :
      { 0x00, 0x40, 0x34, 0x00, 0x00 }, // ;
      { 0x00, 0x08, 0x14, 0x22, 0x41 }, // <
      { 0x14, 0x14, 0x14, 0x14, 0x14 }, // =
      { 0x00, 0x41, 0x22, 0x14, 0x08 }, // >
      { 0x02, 0x01, 0x59, 0x09, 0x06 }, // ?
      { 0x3E, 0x41, 0x5D, 0x59, 0x4E }, // @
      { 0x7C, 0x12, 0x11, 0x12, 0x7C }, // A
      { 0x7F, 0x49, 0x49, 0x49, 0x36 }, // B
      { 0x3E, 0x41, 0x41, 0x41, 0x22 }, // C
      { 0x7F, 0x41, 0x41, 0x41, 0x3E }, // D
      { 0x7F, 0x49, 0x49, 0x49, 0x41 }, // E
      { 0x7F, 0x09, 0x09, 0x09, 0x01 }, // F
      { 0x3E, 0x41, 0x41, 0x51, 0x73 }, // G
      { 0x7F, 0x08, 0x08, 0x08, 0x7F }, // H
      { 0x00, 0x41, 0x7F, 0x41, 0x00 }, // I
      { 0x20, 0x40, 0x41, 0x3F, 0x01 }, // J
      { 0x7F, 0x08, 0x14, 0x22, 0x41 }, // K
      { 0x7F, 0x40, 0x40, 0x40, 0x40 }, // L
      { 0x7F, 0x02, 0x1C, 0x02, 0x7F }, // M
      { 0x7F, 0x04, 0x08, 0x10, 0x7F }, // N
      { 0x3E, 0x41, 0x41, 0x41, 0x3E }, // O
      { 0x7F, 0x09, 0x09, 0x09, 0x06 }, // P
      { 0x3E, 0x41, 0x51, 0x21, 0x5E }, // Q
      { 0x7F, 0x09, 0x19, 0x29, 0x46 }, // R
      { 0x26, 0x49, 0x49, 0x49, 0x32 }, // S
      { 0x03, 0x01, 0x7F, 0x01, 0x03 }, // T
      { 0x3F, 0x40, 0x40, 0x40, 0x3F }, // U
      { 0x1F, 0x20, 0x40, 0x20, 0x1F }, // V
      { 0x3F, 0x40, 0x38, 0x40, 0x3F }, // W
      { 0x63, 0x14, 0x08, 0x14, 0x63 }, // X
      { 0x03, 0x04, 0x78, 0x04, 0x03 }, // Y
      { 0x61, 0x59, 0x49, 0x4D, 0x43 }, // Z
      { 0x00, 0x7F, 0x41, 0x41, 0x41 }, // [
      { 0x02, 0x04, 0x08, 0x10, 0x20 }, // backslash
      { 0x00, 0x41, 0x41, 0x41, 0x7F }, // ]
      { 0x04, 0x02, 0x01, 0x02, 0x04 }, // ^
      { 0x40, 0x40, 0x40, 0x40, 0x40 }, // -
      { 0x00, 0x03, 0x07, 0x08, 0x00 }, // `
      { 0x20, 0x54, 0x54, 0x78, 0x40 }, // a
      { 0x7F, 0x28, 0x44, 0x44, 0x38 }, // b
      { 0x38, 0x44, 0x44, 0x44, 0x28 }, // c
      { 0x38, 0x44, 0x44, 0x28, 0x7F }, // d
      { 0x38, 0x54, 0x54, 0x54, 0x18 }, // e
      { 0x00, 0x08, 0x7E, 0x09, 0x02 }, // f
      { 0x18, 0xA4, 0xA4, 0x9C, 0x78 }, // g
      { 0x7F, 0x08, 0x04, 0x04, 0x78 }, // h
      { 0x00, 0x44, 0x7D, 0x40, 0x00 }, // i
      { 0x20, 0x40, 0x40, 0x3D, 0x00 }, // j
      { 0x7F, 0x10, 0x28, 0x44, 0x00 }, // k
      { 0x00, 0x41, 0x7F, 0x40, 0x00 }, // l
      { 0x7C, 0x04, 0x78, 0x04, 0x78 }, // m
      { 0x7C, 0x08, 0x04, 0x04, 0x78 }, // n
      { 0x38, 0x44, 0x44, 0x44, 0x38 }, // o
      { 0xFC, 0x18, 0x24, 0x24, 0x18 }, // p
      { 0x18, 0x24, 0x24, 0x18, 0xFC }, // q
      { 0x7C, 0x08, 0x04, 0x04, 0x08 }, // r
      { 0x48, 0x54, 0x54, 0x54, 0x24 }, // s
      { 0x04, 0x04, 0x3F, 0x44, 0x24 }, // t
      { 0x3C, 0x40, 0x40, 0x20, 0x7C }, // u
      { 0x1C, 0x20, 0x40, 0x20, 0x1C }, // v
      { 0x3C, 0x40, 0x30, 0x40, 0x3C }, // w
      { 0x44, 0x28, 0x10, 0x28, 0x44 }, // x
      { 0x4C, 0x90, 0x90, 0x90, 0x7C }, // y
      { 0x44, 0x64, 0x54, 0x4C, 0x44 }, // z
      { 0x00, 0x08, 0x36, 0x41, 0x00 }, // {
      { 0x00, 0x00, 0x77, 0x00, 0x00 }, // |
      { 0x00, 0x41, 0x36, 0x08, 0x00 }, // }
      { 0x00, 0x06, 0x09, 0x06, 0x00 },  // degree symbol = '~'
      { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
};
#endif
