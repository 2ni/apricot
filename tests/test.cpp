/*
 * basic functions for our test suite
 */
#include <stdio.h>

#include "test.h"
#include "struct.h"

void printarray(uint8_t *array, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    printf(" %02x", array[i]);
  }
  printf("\n");
}

Test_Outcome is_same(Packet *one, Packet *two) {
  if (one->len != two->len) return FAIL;

  for (uint8_t i=0; i<one->len; i++) {
    if (one->data[i] != two->data[i]) return FAIL;
  }

  return PASS;
}

Test_Outcome validate(const char *name, Packet *expected, Packet *got) {
  Test_Outcome result = is_same(expected, got);

  print_test(name, result);
  if (result == FAIL) {
    printf("   expected: ");
    printarray(expected->data, expected->len);
    printf("   got     : ");
    printarray(got->data, got->len);
  }

  return result;
}

Test_Outcome validate(const char *name, uint8_t expected, uint8_t got) {
  Test_Outcome result = PASS;
  if (expected != got) result = FAIL;

  print_test(name, result);
  if (result == FAIL) {
    printf("   expected: %u\n", expected);
    printf("   got     : %u\n", got);
  }

  return result;
}

void print_test(const char *title, Test_Outcome result) {
  printf(" %s %s\n", result == PASS ? OK("\xE2\x9C\x93") : NOK("\xE2\x9C\x97"), title);
}

Test_Result run_test(Test_Result (*function)()) {
  printf("%s\n", __func__);

  // call test function
  Test_Result result = (*function)();

  // Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}

PORT_t FakePort;
ADC_t FakeADC;
SIGROW_t FakeSIGROW;

/*
 * override pgmspace functions
 */
uint8_t pgm_read_byte(const uint8_t *elm) {
  return *elm;
}

