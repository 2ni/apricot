/*
 * pin configuration settings for forgetmenot
 */

#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "pins.h"

#define touch_is_pressed_bm 0x80
#define touch_short_bm      0x01
#define touch_long_bm       0x02
#define touch_verylong_bm   0x03

class TOUCH {
  public:
    TOUCH(pins_t *pin, uint16_t threshold_low = 15, uint16_t threshold_high = 30, uint16_t threshold = 0);
    uint16_t get_data();
    uint8_t was_pressed();
    uint16_t get_avg();
    uint8_t is_pressed(pins_t *led = 0);

  private:
    void set_thresholds(uint16_t threshold_low, uint16_t threshold_high, uint16_t threshold = 0);
    pins_t *pin;
    uint16_t threshold = 0;
    uint16_t threshold_upper = 0;
    uint16_t threshold_lower = 0;
    uint8_t finger_present = 0;
    uint32_t now = 0;
    // uint8_t press_notified = 0;
};

#endif
