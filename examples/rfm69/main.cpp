#include <util/delay.h>
#include <string.h>

#include "pins.h"
#include "uart.h"
#include "mcu.h"
#include "rfm69.h"

#define NODE 2
#define GATEWAY 99
#define NETWORK 33

int main(void) {
  mcu_init();

  RFM69 rfm69;
  RFM69::Packet response;
  uint8_t counter = 0;
  uint8_t len;
  char msg[8];
  uint16_t vcc;
  uint8_t rssi_limit_reached = 0;
  int8_t pwrchange;
  uint8_t last_rssi;

  uint8_t version = rfm69.init(get_deviceid(), NETWORK);
  if (!version) {
    DL("rfm69 init failed");
  } else {
    DF("RFM69 Version: 0x%02x\n", version);
  }


  rfm69.set_power_level(10);
  while (1) {
    vcc = get_vin();
    len = 5;
    msg[0] = (0x00<<4) | 0x01; // type: dbg (0x00), 1byte
    msg[1] = counter;
    msg[2] = (0x01<<4) | 0x02; // type: vcc (0x01), 2bytes
    msg[3] = vcc >> 8;
    msg[4] = vcc & 0xff;

    if (rssi_limit_reached) {
      rssi_limit_reached = 0;
      msg[5] = (0x03<<4) | 0x02; // type: rssi (0x03), 2bytes
      msg[6] = 0x80;
      msg[7] = last_rssi;
      len += 3;
    }

    DF("sending %03u | ", counter);
    if (rfm69.send_retry(GATEWAY, msg, len, &response, 2)) {
      last_rssi = (uint8_t)(-response.rssi);
      // DF(OK("from 0x%06lX: '%s' (%idBm)") "\n", response.from, response.payload, response.rssi);
      DF("ack (%idBm) | ", response.rssi);
      uart_arr("raw response", response.payload, response.len, 0);
      D(" | ");
      uint8_t i = 0;
      uint8_t first_byte;
      uint32_t timestamp;
      while (i < response.len) {
        first_byte = response.payload[i];
        uint8_t data_type = first_byte >> 4;
        uint8_t data_len = first_byte & 0x0f;
        i++; // 1st byte is type/len
        switch(data_type) {
          case 0x01: // case timestamp
            timestamp = ((uint32_t)response.payload[i] << 24)
              | ((uint32_t)response.payload[i+1] << 16)
              | ((uint16_t)response.payload[i+2] << 8)
              | response.payload[i+3];

            DF(OK("timestamp: %lu") " | ", timestamp);
            break;
          case 0x03: // type: rssi (0x03), 1 byte
            pwrchange = response.payload[i] & 0x0f | (response.payload[i] & 0x08 ? 0xf0 : 0x00); // convert int4_t to int8_t
            rssi_limit_reached = !rfm69.set_power_level_relative(pwrchange);
            DF("pwr: %i -> %u  (%u) | ", pwrchange, rfm69.get_power_level(), rssi_limit_reached);
            break;
          default:
            DF(NOK("unknown type 0x%02x") " | ", data_type);
            break;
        }
        i += data_len;
      }
      DL("");
    } else {
      DL(NOK("no response"));
    }
    rfm69.sleep();

    counter++;
    clock.sleep_for(8192);
  }
}
