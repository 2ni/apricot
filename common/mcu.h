#ifndef __MCU__
#define __MCU__

#include <avr/io.h>
#include "clock.h"

extern CLOCK clock;

void mcu_init();
uint16_t get_vin();
uint16_t get_vcc_mcu();

#endif
