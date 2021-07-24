/*
 * pin configuration settings for forgetmenot
 */

#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "pins.h"

class TOUCH {
  public:
    typedef enum {
      NONE = 0x00,
      SHORT = 0x01,
      LONG = 0x02,
      VERYLONG = 0x04
    } Press_type;

    TOUCH(pins_t *pin);
    void init(uint16_t low = 10, uint16_t high = 100, uint16_t idle = 0);
    uint16_t get_data();
    uint16_t get_avg();
    // 12288*8/32768 = 3sec, 20480*8/32768 = 5sec
    uint8_t is_pressed(void (*fn)(Press_type, uint32_t), uint32_t tick_long, uint32_t tick_verylong);
    uint8_t is_pressed(void (*fn)(Press_type, uint32_t) = 0, pins_t *led = &pins_led, uint32_t tick_long = 12288, uint32_t tick_verylong = 20480);

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
