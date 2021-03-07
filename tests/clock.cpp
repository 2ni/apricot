#include "clock.h"
#include "uart.h"

CLOCK* CLOCK::clock_ptr;

CLOCK::CLOCK() {
  clock_ptr = this;
  _is_running = 0;
  _mode = CONTINUOUS;
}

void CLOCK::init(uint16_t per) {
  current_tick = 0;
  _is_running = 1;
}

uint8_t CLOCK::is_continuous() {
  return _mode == CONTINUOUS && _is_running;
}

void CLOCK::start() {
  _is_running = 1;
}

void CLOCK::stop() {
  _is_running = 0;
}

void CLOCK::sleep_until(uint32_t tick_until) {
  _mode = CONTINUOUS;
}

void CLOCK::sleep_for(uint32_t ticks) {
  sleep_until(current_tick + ticks);
}

uint32_t CLOCK::ms2ticks(uint32_t ms) {
  return ms * 32768/1000/8;
}

uint32_t CLOCK::ticks2ms(uint32_t ticks) {
  return (ticks * 8*1000)/32768;
}

void CLOCK::sleep_once(uint16_t per, uint16_t prescaler) {
  _mode = SINGLE;
  _is_running = 0;
}

void CLOCK::sleep_once(uint16_t duration, Duration_type ms_or_s) {
  if (ms_or_s == MSEC) {
    sleep_once(((uint32_t)duration*1024)/1000, 5);
  } else if (ms_or_s == SEC) {
    sleep_once(duration, 15);
  }
}
