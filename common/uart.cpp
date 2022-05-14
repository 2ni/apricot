/*
 * library to send debug information to UART
 * alternate usart pin must be set (PORTMUX.CTRLB)
 * TX=PA1
 * RX=PA2
 *
 * ring buffer based on https://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Der_UART
 */

#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"

#define TX_BUFF_SIZE 64
static uint8_t tx_buff[TX_BUFF_SIZE];
static uint8_t tx_in; // pointer of filling buffer
volatile uint8_t tx_out; // pointer of sending

// TEST defined in Makefile of tests/
#ifndef TEST
ISR(USART0_DRE_vect) {
  // nothing to send
  if (tx_in == tx_out) {
    USART0.CTRLA &= ~USART_DREIE_bm;
    return;
  }

  USART0.TXDATAL = tx_buff[tx_out];
  USART0.STATUS = USART_TXCIF_bm; // clear txcif
  uart_rollover(&tx_out, TX_BUFF_SIZE);
}
#endif

/*
 * the ISR needs to be setup in the main code for now
 *
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  pins_flash(&pins_led, 1, 100);
}
*/

/*
 * setup uart and tx pin
 *
 * if rx enabled, be sure to setup isr in main code:
 * ISR(USART0_RXC_vect) {
 *   uint8_t in = USART0.RXDATAL;
 * }
 *
 */
void uart_init(uint8_t enable_rx) {
  sei();
  USART0.BAUD = (uint16_t)USART_BAUD_RATE(USART_BPS);
  USART0.CTRLB = USART_TXEN_bm;  // enable TX
  USART_PORT.DIRSET = USART_TX;
  USART_PORT.DIRCLR = USART_RX;
  tx_in = 0;
  tx_out = 0;

  _delay_ms(600); // the key listener needs some time to start

  // see https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences for colors
  DF("\033[1;38;5;18;48;5;226m Hello from 0x%06lX @ %uMHz \033[0m\n", get_deviceid(), (uint8_t)(F_CPU/1000000));

  if (enable_rx) {
    USART0.CTRLB |= USART_RXEN_bm;  // enable RX
    USART0.CTRLA |= USART_RXCIE_bm; // enable RX interrupt
    USART_PORT.DIRCLR = USART_RX;
    DL("rx enabled");
  }
}

/*
 * send the variable name of a tuple to the uart
 */
void _uart_send_tuple_key(const char* key) {
  uart_send_string_p(key);
  uart_send_string_p(PSTR(": "));
}

/*
 * sends a tuple with type const char to the uart
 * ie "key: <const char>"
 */
void uart_tuple(const char* key, const char* value) {
  _uart_send_tuple_key(key);
  uart_send_string_p(value);
  uart_send_string_p(PSTR("\r\n"));
}

/*
 * sends a tuple with type uint16_t to the uart
 * ie "key: <uint16_t>"
 *
 * if hex (base=16) chosen, the output is shown as 0xAAAA
 */
void uart_tuple(const char* key, uint16_t value, uint8_t base) {
  _uart_send_tuple_key(key);
  char buf[6];
  if (base==16) {
    sprintf(buf, "0x%02x", value);
  } else {
    itoa(value, buf, base);
  }

  uart_send_string(buf);
  uart_send_string_p(PSTR("\r\n"));
}

void uart_tuple(const char *key, uint8_t value, uint8_t base) {
  _uart_send_tuple_key(key);
  char buf[9];
  if (base==16) {
    sprintf(buf, "0x%02x", value);
  } else if (base==2) {
    for (uint8_t i=0; i<8; i++) {
      buf[i] = value & 0x80 ? '1' : '0';
      value <<= 1;
    }
  } else {
    itoa(value, buf, base);
  }

  uart_send_string(buf);
  uart_send_string_p(PSTR("\r\n"));
}

/*
 * sends a tuple with type const char to the uart
 * ie "key: <char array>"
 */
void uart_tuple(const char* key, char* value) {
  _uart_send_tuple_key(key);
  uart_send_string(value);
  uart_send_string_p(PSTR("\r\n"));
}

/*
 * sends a single char to the uart
 */
void uart_send_char(unsigned char c) {
  uint8_t next = tx_in;
  uart_rollover(&next, TX_BUFF_SIZE);
  while (next == tx_out);
  tx_buff[tx_in] = c;
  // wait until at least one byte free
  tx_in = next;
  USART0.CTRLA |= USART_DREIE_bm;

  /*
  USART0.STATUS = USART_TXCIF_bm; // clear flag
  while (!(USART0.STATUS & USART_DREIF_bm));
  USART0.TXDATAL = c;
  while (!(USART0.STATUS & USART_TXCIF_bm)); // wait until data has been sent to avoid issues with eg sleep
  */
}

uint8_t uart_is_busy() {
  return (!(USART0.STATUS & USART_TXCIF_bm));
}

/*
 * sends a char array to the uart
 * ie "key: <const char>"
 */
void uart_send_string(char* s) {
  while (*s) uart_send_char(*s++);
}

/*
 * sends a const char array to the uart
 */
void uart_send_string_p(const char* s) {
  while (pgm_read_byte(s)) uart_send_char(pgm_read_byte(s++));
}

/*
 * sends a uint16_t to the uart
 */
void uart_send_digit(uint16_t value, uint8_t base) {
  char buf[6];
  itoa(value, buf, base);
  uart_send_string(buf);
}

/*
 * convert an uint to a char array
 * eg 345 -> "3.45"
 *
 * returns length of string
 */
uint8_t uart_u2c(char *buf, uint16_t value, uint8_t precision) {
  itoa(value, buf, 10);
  uint8_t len = strlen(buf);
  uint8_t wrote_dot = 0;

  // ensure char array is at least precision+1 length -> fill it with 0
  int8_t p_read = len - 1;
  uint8_t p_write;
  if (precision == 0) {
    p_write = len - 1;
  } else if (precision >= len) {
    p_write = precision + 1;
  } else {
    p_write = len;
  }
  /*
  DT_I("len", len);
  DT_I("precision", precision);
  DT_I("p_read", p_read);
  DT_I("p_write", p_write);
  */

  if (precision) {
    for (int8_t i=p_write; i>=0; i--) {
      // DF("i:%i p_read:%i\n", i, p_read);
      if (!wrote_dot && precision == 0) {
        wrote_dot = 1;
        buf[i] = '.';
      }
      else if (p_read < 0) {
        buf[i] = '0';
      } else {
        buf[i] = buf[p_read];
        p_read--;
      }

      precision--;
    }
  }
  buf[p_write+1] = '\0';

  return p_write + 1;
}

/*
 * convert seconds to human readable char array
 * max 2^16 = 18h12m16s
 */
uint8_t  uart_sec2human(char *buf, uint16_t secs) {
  uint8_t minutes = (secs / 60) % 60;
  uint8_t hours = secs / 3600;
  uint8_t seconds = secs % 60;

  if (hours) return sprintf(buf, "%02uh%02um%02us", hours, minutes, seconds);
  else if (minutes)  return sprintf(buf, "%02um%02us", minutes, seconds);
  else return sprintf(buf, "%02us", seconds);
}

/*
 * fast implemention to print arrays
 * (avoid using any printf call)
 */
void uart_arr(const char *name, uint8_t *arr, uint8_t len, uint8_t newline) {
  const char *name_start = name;
  while (*name) uart_send_char(*name++);
  if (name_start != name) {
    uart_send_char(':');
    uart_send_char(' ');
  }

  for (uint8_t i=0; i<len; i++) {
    uint8_t hi = (arr[i] >> 4) & 0xf;
    uint8_t lo = arr[i] & 0xf;
    uart_send_char(hi < 10 ? '0' + hi : ('a' + hi - 10));
    uart_send_char(lo < 10 ? '0' + lo : ('a' + lo - 10));
    if (i<(len-1)) uart_send_char(' ');
  }
  if (newline) uart_send_char('\n');
}

/*
 * increment value and wrap around if > max
 * (for ring buffer)
 */
void uart_rollover(uint8_t *value, uint8_t max) {
  *value = ++*value >= max ? 0 : *value;
}

void uart_rollover(volatile uint8_t *value, uint8_t max) {
  *value = ++*value >= max ? 0 : *value;
}

/*
 * decrement value and wrap around if < 0
 * (for ring buffer)
 */
void uart_rollbefore(uint8_t *value, uint8_t max) {
  *value = *value == 0 ? max - 1 : *value - 1;
}

void uart_rollbefore(volatile uint8_t *value, uint8_t max) {
  *value = *value == 0 ? max - 1 : *value - 1;
}

/*
 *  get 24bit device id
 *  can be printed with DF("Hello from 0x%06lX", get_deviceid());
 */
uint32_t get_deviceid() {
  return ((uint32_t)SIGROW.DEVICEID0<<16) | ((uint16_t)SIGROW.DEVICEID1<<8) | (uint8_t)SIGROW.DEVICEID2;
}
