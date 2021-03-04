/*
 * pin configuration settings for forgetmenot
 */

#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "pins.h"

#define touch_is_pressed_bm 0x80
#define touch_short_bm      0x01
#define touch_long_bm       0x02
#define touch_verylong_bm   0x04

class TOUCH {
  public:
    typedef enum {
      NONE = 0x00,
      SHORT = 0x01,
      LONG = 0x02,
      VERYLONG = 0x04
    } Press_type;

    TOUCH(pins_t *pin, uint16_t threshold_low = 10, uint16_t threshold_high = 50);
    uint16_t get_data();
    uint16_t get_avg();
    uint8_t is_pressed(void (*fn)(Press_type, uint32_t) = 0, pins_t *led = &pins_led);

  private:
    void set_thresholds(uint16_t threshold_low, uint16_t threshold_high);
    pins_t *pin;
    uint16_t threshold_upper = 0;
    uint16_t threshold_lower = 0;
    uint8_t occupied = 0;
    uint8_t pressed = 0;
    uint32_t now = 0;
    uint32_t start_tick = 0;
    Press_type type = NONE;
};

#endif
