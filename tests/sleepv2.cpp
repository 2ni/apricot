#include "sleepv2.h"
#include "uart.h"

SLEEP* SLEEP::sleep_ptr;

SLEEP::SLEEP() {
  sleep_ptr = this;
  _is_running = 0;
  _mode = CONTINUOUS;
}

void SLEEP::init(uint16_t per) {
  current_tick = 0;
  _is_running = 1;
}

uint8_t SLEEP::is_continuous() {
  return _mode == CONTINUOUS && _is_running;
}

void SLEEP::start() {
  _is_running = 1;
}

void SLEEP::stop() {
  _is_running = 0;
}

void SLEEP::sleep_until(uint32_t tick_until) {
  _mode = CONTINUOUS;
}

void SLEEP::sleep_for(uint32_t ticks) {
  sleep_until(current_tick + ticks);
}

uint32_t SLEEP::ms2ticks(uint32_t ms) {
  return ms * 32768/1000/8;
}

uint32_t SLEEP::ticks2ms(uint32_t ticks) {
  return (ticks * 8*1000)/32768;
}

void SLEEP::sleep_once(uint16_t per, uint16_t prescaler) {
  _mode = SINGLE;
  _is_running = 0;
}

void SLEEP::sleep_once(uint16_t duration, Duration_type ms_or_s) {
  if (ms_or_s == MSEC) {
    sleep_once(((uint32_t)duration*1024)/1000, 5);
  } else if (ms_or_s == SEC) {
    sleep_once(duration, 15);
  }
}
