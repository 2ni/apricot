#ifndef __PGMSPACE_H_
#define __PGMSPACE_H_

#include <stdint.h>

#define PROGMEM
#define PSTR

uint8_t pgm_read_byte(const uint8_t *elm);
uint8_t pgm_read_byte(const char *elm);

#endif
