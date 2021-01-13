/*
 * unit test cases for lorawan
 */
#include <stdio.h>
#include <avr/pgmspace.h>

#include "test.h"
#include "struct.h"
#include "aes.h"
#include "tests_pgmspace.h"

Test_Result tests_pgmspace() {
  printf("%s\n", __func__);

  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;

  // tests
  number_of_tests++;
  number_of_passed += validate("read byte", 0x63, pgm_read_byte(&AES_BOX[0][0]));

  // final
  Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}
