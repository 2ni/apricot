#ifndef __TEST_H__
#define __TEST_H__

#include <stdio.h>
#include <avr/io.h>

#include "lorawan_struct.h"

#define NOK(str)  "\033[31;1m" str "\033[0m"  // output in red
#define OK(str)   "\033[32;1m" str "\033[0m"  // output in green
#define WARN(str) "\033[33;1m" str "\033[0m"  // output in yellow
#define BOLD(str) "\033[1m" str "\033[0m"     // output bold

typedef enum {
  FAIL,
  PASS
} Test_Outcome;

typedef struct {
  uint16_t total;
  uint16_t passed;
} Test_Result;

void printarray(uint8_t *array, uint8_t len);
void printarray(uint32_t *array, uint8_t len);
Test_Outcome is_same(Packet *one, Packet *two);
Test_Outcome validate(const char *name, Packet *expected, Packet *got);
Test_Outcome validate(const char *name, uint16_t expected, uint16_t got);
void print_test(const char *title, Test_Outcome result);
Test_Result run_test(Test_Result (*function)());

#endif
