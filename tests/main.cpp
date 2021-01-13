/*
 * gcc -DF_CPU=10000000 -std=c++11 -I mocks -I ../common ../common/pins.cpp ../common/cmac.cpp ../common/aes.cpp ../common/lorawan.cpp ../common/rfm95.cpp spi.cpp sleep.cpp uart.cpp millis.cpp delay.cpp test.cpp main.cpp -o test && ./test
 *
 */
#include <stdio.h>

#include "test.h"
#include "struct.h"

#include "tests_lora.h"
#include "tests_pgmspace.h"

uint32_t m; // time counter used in lorawan.cpp

int main() {
  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;
  Test_Outcome outcome;
  Test_Result test_result;

  // test pgmspace mock
  test_result = tests_pgmspace();
  number_of_passed += test_result.passed;
  number_of_tests += test_result.total;

  // test B0 appending
  test_result = tests_lora();
  number_of_passed += test_result.passed;
  number_of_tests += test_result.total;

  // summary
  uint16_t number_of_failed = number_of_tests-number_of_passed;
  printf("\n");
  if (number_of_failed) {
    printf(NOK("Failures!!!: %u/%u.") "\n", number_of_failed, number_of_tests);
  } else {
    printf(OK("All good: %u/%u.") "\n", number_of_passed, number_of_tests);
  }

  return number_of_failed;
}
