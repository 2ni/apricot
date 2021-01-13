#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"

#define PSTR

/*
 * setup uart and tx pin
 */
void uart_setup(void) {
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
    sprintf(buf, "%d", value);
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
  printf("%c", c);
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
  printf("%s", s);
}

/*
 * sends a uint16_t to the uart
 */
void uart_send_digit(uint16_t value, uint8_t base) {
  char buf[6];
  if (base==10) {
    sprintf(buf, "%d", value);
  } else {
    sprintf(buf, "%02x", value);
  }
  uart_send_string(buf);
}

/*
 * convert an uint to a char array
 * eg 345 -> "3.45"
 */
void uart_u2c(char *buf, uint16_t value, uint8_t precision) {
  sprintf(buf, "%d", value);
  uint8_t len = strlen(buf);
  uint8_t wrote_dot = 0;

  // ensure char array is at least precision+1 length -> fill it with 0
  int8_t p_read = len - 1;
  uint8_t p_write;
  if (precision >= len) {
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
  buf[p_write+1] = '\0';
}


void uart_arr(const char *name, uint8_t *arr, uint8_t len) {
  DF("%s:", name);
  for(uint8_t i=0; i<len; i++) {
    DF(" %02x", arr[i]);
  }
  DL("");
}
