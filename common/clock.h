/*
 *
 */
#ifndef __CLOCK_H__
#define __CLOCK_H__
#include <avr/io.h>

class CLOCK {
  public:
    typedef enum {
      MSEC,
      SEC
    } Duration_type;

    typedef enum {
      SINGLE,
      CONTINUOUS
    } Mode;

    CLOCK();
    void     init(uint16_t per = 7);
    void     start();
    void     stop();
    uint8_t  is_continuous();
    void     sleep_until(uint32_t tick_until);
    void     sleep_for(uint32_t ticks);
    uint32_t ms2ticks(uint32_t ms);
    uint32_t ticks2ms(uint32_t ticks);
    void     sleep_once(uint16_t per, uint16_t prescaler);
    void     sleep_once(uint16_t duration, Duration_type ms_or_s);

    volatile uint32_t current_tick;
    static CLOCK *clock_ptr;

  private:
    uint8_t _is_running;
    Mode    _mode;
};

#endif
