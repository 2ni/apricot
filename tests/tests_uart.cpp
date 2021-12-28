#include <stdio.h>
#include <string.h>

#include "test.h"
#include "tests_uart.h"
#include "uart.h"

Test_Result tests_uart() {
  printf("%s\n", __func__);

  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;
  char buf[10] = {0};
  uint8_t len = 0;

  // uart_u2c
  len = uart_u2c(buf, 121, 1);
  number_of_passed += validate("uart_u2c (12.1)", "12.1", buf);
  number_of_passed += validate("uart_u2c len", 4, len);

  len = uart_u2c(buf, 123, 0);
  number_of_passed += validate("uart_u2c (123)", "123", buf);
  number_of_passed += validate("uart_u2c len", 3, len);

  len = uart_u2c(buf, 12, 2);
  number_of_passed += validate("uart_u2c (0.12)", "0.12", buf);
  number_of_passed += validate("uart_u2c len", 4, len);
  number_of_tests += 6;

  // uart_sec2human
  len = uart_sec2human(buf, 10);
  number_of_passed += validate("sec2human (10s)", "10s", buf);

  len = uart_sec2human(buf, 12253);
  number_of_passed += validate("sec2human (03h24m13s)", "03h24m13s", buf);

  len = uart_sec2human(buf, 359);
  number_of_passed += validate("sec2human (05m59s)", "05m59s", buf);

  uint32_t send_data_at = 3691486;
  uint32_t now = 17774;
  len = uart_sec2human(buf, (((send_data_at - now)) * (7 + 1)) / 32768);
  number_of_passed += validate("sec2human (14m59s)", "14m56s", buf);

  number_of_tests += 4;

  // final
  Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}
