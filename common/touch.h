/*
 * pin configuration settings for forgetmenot
 */

#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "pins.h"

class TOUCH {
  public:
    enum STATUS {
      IDLE = 0,
      PRESSED = 1,
      SHORT = 2,
      LONG = 3,
    };
    TOUCH(pins_t *pin);
    TOUCH(pins_t *pin, uint16_t threshold);
    uint16_t get_data();
    uint8_t is_pressed();
    uint8_t was_pressed();
    uint16_t get_avg();
    STATUS pressed(uint16_t timeout = 3000, pins_t *led = 0);

  private:
    void set_thresholds(uint16_t threshold);
    pins_t *pin;
    uint16_t threshold = 0;
    uint16_t threshold_upper = 0;
    uint16_t threshold_lower = 0;
    uint8_t finger_present = 0;
    uint8_t status = IDLE;
    uint32_t now = 0;
    // uint8_t press_notified = 0;
    uint8_t status_finger = 0;
};

#endif
