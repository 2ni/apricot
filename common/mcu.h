#ifndef __MCU__
#define __MCU__

#include <avr/io.h>
#include "sleepv2.h"

extern SLEEP sleep;

void mcu_init();
uint16_t get_vin();
uint16_t get_vcc_mcu();

#endif
