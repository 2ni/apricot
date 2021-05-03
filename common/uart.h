/*
 * simple bitbanging (software) UART to debug output
 * write only
 *
 * TODO: use clk interrupts, eg https://github.com/MarcelMG/AVR8_BitBang_UART_TX/blob/master/main.c
 * TODO: use USART TxD (PB2) from chip instead of software solution
 * inspidred by
 * - http://www.justgeek.de/a-simple-simplex-uart-for-debugging-avrs/
 * - https://github.com/MartinD-CZ/AVR-Li-Ion-charger/blob/master/firmware/ATtiny%20USB%20charger/
 */
#ifndef __UART_H__
#define __UART_H__
#include <avr/pgmspace.h> // for PSTR
#include <stdio.h>
#include <avr/io.h>

#define USART_BPS   19200
#define USART_BAUD_RATE(BAUD_RATE) ((float)(10000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#ifdef __AVR_ATtiny3217__

#define USART_PORT PORTA
#define USART_TX PIN1_bm
#define USART_RX PIN2_bm

#elif defined(__AVR_ATtiny1604__)

#define USART_PORT PORTB
#define USART_TX PIN2_bm
#define USART_RX PIN3_bm

#endif

// DEBUG set in Makefile
#ifdef DEBUG
  #define DINIT()            uart_init()
  #define DLF()              uart_send_string_p(PSTR("\n"))
  #define D(str)             uart_send_string_p(PSTR(str))
  #define DL(str)            { uart_send_string_p(PSTR(str)); DLF(); }
  #define DF(format, ...)    { char uart_buf[120]; sprintf(uart_buf, format, __VA_ARGS__); uart_send_string(uart_buf); }
  #define DT_C(key, value)   uart_tuple(PSTR(key), PSTR(value))
  #define DT_S(key, value)   uart_tuple(PSTR(key), value)
  #define DT_I(key, value)   uart_tuple(PSTR(key), value)
  #define DT_IH(key, value)  uart_tuple(PSTR(key), value, 16)
  // https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences
  // https://stackoverflow.com/questions/3219393/stdlib-and-colored-output-in-c
  #define NOK(str)           "\033[31;1m" str "\033[0m"  // output in red
  #define OK(str)            "\033[32;1m" str "\033[0m"  // output in green
  #define WARN(str)          "\033[33;1m" str "\033[0m"  // output in yellow
  #define BOLD(str)          "\033[1m" str "\033[0m"     // output bold
#else
  #define DINIT()
  #define D(str)
  #define DLF()
  #define DL(str)
  #define DF(size, format, ...)
  #define DT_C(key, value)
  #define DT_S(key, value)
  #define DT_I(key, value)
  #define DT_IH(key, value)
#endif

void uart_init();
void uart_tuple(const char* key, const char* value);
void uart_tuple(const char* key, uint16_t value, uint8_t base=10);
void uart_tuple(const char* key, char* value);
void uart_send_char(unsigned char c);
void uart_send_string(char* s);
void uart_send_string_p(const char* s);
void uart_send_digit(uint16_t value, uint8_t base=10);
void uart_u2c(char *buf, uint16_t value, uint8_t precision=2);
void uart_arr(const char *name, uint8_t *arr, uint8_t len);
uint32_t get_deviceid();

#endif
