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

uint32_t millis_time() {
  return 1234;
}
