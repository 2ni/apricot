/*
 *
 * provide a reliable timer which is constantly running
 *
 */
#include <stdint.h>

volatile uint32_t millis;

void millis_init(uint32_t cpu) {
}

void millis_init() {
}

uint8_t millis_is_init() {
  return 1;
}

uint32_t millis_time() {
  return 1234;
}
