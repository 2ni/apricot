#ifndef __EEPROM_H_
#define __EEPROM_H_

#include <stddef.h>     /* size_t */
#include <stdint.h>

void eeprom_update_block (const void *__src, void *__dst, size_t __n);
void eeprom_read_block (void *__dst, const void *__src, size_t __n);

#endif
