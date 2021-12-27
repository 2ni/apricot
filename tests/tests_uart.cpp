#include <stdio.h>
#include <string.h>

#include "test.h"
#include "tests_uart.h"
#include "uart.h"

Test_Result tests_uart() {
  printf("%s\n", __func__);

  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;
  char buf[5] = {0};

  // tests
  uart_u2c(buf, 121, 1);
  number_of_passed += validate("uart_u2c (precision: 1)", "12.1", buf);

  uart_u2c(buf, 123, 0);
  number_of_passed += validate("uart_u2c (precision: 0)", "123", buf);

  uart_u2c(buf, 12, 2);
  number_of_passed += validate("uart_u2c (precision: 2)", "0.12", buf);
  number_of_tests += 3;

  // final
  Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}
