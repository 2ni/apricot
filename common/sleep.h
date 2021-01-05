/*
 * simple sleep mode
 * code based on https://www.avrfreaks.net/forum/tinyavr-1-series-071ua-rtc-running-how
 *
 * in standby mode periphery can be activated by settings RUNSTDBY in the corresponding periphery
 * ports stay on in sleep mode
 *
 * modes: idle, standby, powerdown
 *
 * RTC needs standby mode
 *
 * !!!!!! CAUTION !!!!!!
 * be sure to call clear_all_pins to get real low power and activate only the pins you need
 *
 * power consumption:
 *   ~3uA : power directly 3.3v (w/o LDO module)
 *   ?: power via LDO module
 *
 */
#ifndef __SLEEP_H__
#define __SLEEP_H__
#include <avr/io.h>
#include <avr/interrupt.h>

void _s_sleep(uint16_t per, uint8_t prescaler);
void sleep_ms(uint16_t ms);
void sleep_s(uint16_t s);

#endif
