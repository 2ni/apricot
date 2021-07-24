#include "uart.h"
#include "mcu.h"
#include "humidity_sensor.H"
#include "touch.h"

HUMIDITYSENSOR sensor(&PA5, &PA6, &PA3);
TOUCH button(&PC3);

void button_callback(TOUCH::Press_type type, uint32_t ticks) {
  // DF("press: %s for %lus\n", type == TOUCH::SHORT ? "SHORT" : "LONG", clock.ticks2ms(ticks));
  uint16_t t = sensor.get_temperature();
  uint16_t h = sensor.get_humidity();
  DF("humidity: %u%% (%u) temp: %u vin: %u\n",
    sensor.to_relative(h, sensor.characteristics_humidity, sensor.characteristics_humidity_len),
    h,
    t,
    get_vin()
  );
}

int main(void) {
  mcu_init();
  button.init();

  DL("touch PC3 to measure");

  while (1) {
    button.is_pressed(&button_callback);
    clock.sleep_for(205); // sleep for 50ms: 50*32768/8/1000 = 204.8
  }
}
