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
  uint8_t msg[10];
  uint16_t vcc;
  uint8_t rssi_limit_reached = 0;
  int8_t pwrchange;
  uint8_t rssi_last = 0;
  uint8_t rssi_reset = 1; // ensure we start from scratch when powering up
  uint8_t rssi_request = 0;
  uint8_t i;
  uint8_t fail_count = 0;

  uint8_t version = rfm69.init(get_deviceid(), NETWORK);
  if (!version) {
    DL("rfm69 init failed");
  } else {
    DF("RFM69 Version: 0x%02x\n", version);
  }

  while (1) {
    vcc = get_vin();
    len = 5;
    i = 0;

    DF("sending %03u | pwr: %u | ", counter, rfm69.get_power_level());
    msg[i++] = (0x00<<4) | 0x01; // type: dbg (0x00), 1byte
    msg[i++] = counter;
    msg[i++] = (0x01<<4) | 0x02; // type: vcc (0x01), 2bytes
    msg[i++] = vcc >> 8;
    msg[i++] = vcc & 0xff;

    if (rssi_limit_reached || rssi_reset || rssi_request) {
      DF("lim: %u res: %u requ: %u rssi: %udBm | ", rssi_limit_reached?1:0, rssi_reset?1:0, rssi_request?1:0, rssi_last);
      msg[i++] = (0x03<<4) | 0x02; // type: rssi (0x03), 2bytes
      msg[i++] = (rssi_limit_reached ? 0x80 : 0x00) | (rssi_reset ? 0x40 : 0x00) | (rssi_request ? 0x20 : 0x00) ;  // ctrl (limit, reset, request)
      msg[i++] = rssi_last;
      len += 3;
      rssi_limit_reached = 0;
      rssi_request = 0;
    }
    uart_arr("raw send", msg, len, 0);
    DL("");

    if (rfm69.send_retry(GATEWAY, msg, len, &response, 2)) {
      rssi_reset = 0;
      fail_count = 5;
      rssi_last = (uint8_t)(-response.rssi);
      // DF(OK("from 0x%06lX: '%s' (%idBm)") "\n", response.from, response.payload, response.rssi);
      DF("  " OK("ack") " (%idBm) | ", response.rssi);
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
            pwrchange = (response.payload[i] & 0x0f) | (response.payload[i] & 0x08 ? 0xf0 : 0x00); // convert int4_t to int8_t
            rssi_request = response.payload[i] & 0x20;
            rssi_limit_reached = pwrchange && !rfm69.set_power_level_relative(pwrchange);
            DF("pwr change: %i rssi_request: %i | ", pwrchange, rssi_request?1:0);
            break;
          default:
            DF(NOK("unknown type 0x%02x") " | ", data_type);
            break;
        }
        i += data_len;
      }
      uart_arr("raw", response.payload, response.len, 0);
      DL("");
    } else {
      D(NOK("no response | "));
      if (--fail_count == 0) {
        // failed receiving data for more than x times
        rssi_reset = 1;
        rfm69.set_power_level(23);
        D("reset power | ");
      }
      DL("");
    }
    rfm69.sleep();

    counter++;
    clock.sleep_for(8192);
  }
}
