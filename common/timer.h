/*
 * simple timer class
 */

#ifndef __TIMER_H__
#define __TIMER_H__


class TIMER {
  public:
    TIMER(uint32_t mhz);
    TIMER();
    void start(uint16_t ms);
    void stop();
    uint8_t timed_out();
    static TIMER* timer_pointer;
    volatile uint8_t _timeout = 0;
    volatile uint8_t _rounds;
    uint32_t _last_round_ticks;

  private:
    void init();
    uint32_t _mhz;
};

#endif
