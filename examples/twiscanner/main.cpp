#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "twi.h"
#include "mcu.h"


int main(void) {
  mcu_init();

  twi_init();

  uint8_t data[3] = {0};

  // read raw data from an SHT20
  // see https://github.com/DFRobot/DFRobot_SHT20/blob/master/DFRobot_SHT20.cpp
  uint8_t status = twi_read_bytes(0x40, data, 0xe3, 2); // read temp hold
  if (status == 0) {
    uint32_t raw = ((uint32_t)data[0]<<8) | (uint32_t)data[1];
    char out[6];
    // DF("%u %u\n", data[0], data[1]);
    // DF("temp raw: %lu\n", raw_temp);
    raw *= 17572;
    raw /= 65536;
    raw -= 4685;
    uart_u2c(out, raw, 2);
    DF("temp: %sC\n", out);

    twi_read_bytes(0x40, data, 0xe5, 2); // read humidity hold
    raw = ((uint32_t)data[0]<<8) | (uint32_t)data[1];
    raw *= 12500;
    raw /= 65536;
    raw -= 6;
    uart_u2c(out, raw, 2);
    DF("humidity: %s%%\n", out);
  } else {
    DF("error: %u\n", status);
  }


  DL("start scanning");
  for (uint8_t addr = 0; addr<128; addr+=1) {
    // DF("0x%02x: %s\n", addr, twi_start(addr) ? "-" : "yes");
    if (!twi_start(addr)) {
      DF("0x%02x\n", addr);
    }
    twi_stop();
  }

  DL("done.");
}
