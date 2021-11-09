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
  uint8_t counter = 0;
  uint8_t len;
  char msg[9];
  uint16_t vcc = 0;

  uint8_t version = rfm69.init(get_deviceid(), NETWORK);
  if (!version) {
    DL("rfm69 init failed");
  } else {
    DF("RFM69 Version: 0x%02x\n", version);
  }

  RFM69::Packet response;

  /*
  DL("Waiting for data...");
  while (1) {
    if (rfm69.listen(&response, 0)) { // no timeout
      DF("received: '%s' (%idBm)\n", response.message, response.rssi);
    }
  }
  //*/

  /*
  while (1) {
    DF("sending %03u | ", counter);
    len = sprintf(msg, "%u", counter);
    rfm69.send_frame(GATEWAY, msg, len, 1, 0);
    if (rfm69.listen(&response)) {
      DF("ack (%idBm)", response.rssi);
    }
    DL("");
    counter++;
    clock.sleep_for(4096);
  }
  //*/

  len = 5;
  while (1) {
    vcc = get_vin();
    msg[0] = (0x00<<4) | 0x01; // type: dbg (0x01), 1byte
    msg[1] = counter;
    msg[2] = (0x08<<4) | 0x02; // type: vcc (0x08), 2bytes
    msg[3] = vcc >> 8;
    msg[4] = vcc & 0xff;

    DF("sending %03u | ", counter);
    // len = sprintf(msg, "%u", counter);
    if (rfm69.send_retry(GATEWAY, msg, len, &response, 2)) {
      D("ack | ");
      uart_arr("raw response", response.message, response.len, 0);
      D(" | ");
      // DF(OK("from 0x%06lX: '%s' (%idBm)") "\n", response.from, response.message, response.rssi);
      uint8_t i = 0;
      uint8_t first_byte;
      uint32_t timestamp;
      while (i < response.len) {
        first_byte = response.message[i];
        uint8_t data_type = first_byte >> 4;
        uint8_t data_len = first_byte & 0x0f;
        i++; // 1st byte is type/len
        switch(data_type) {
          case 0x01: // case timestamp
            timestamp = ((uint32_t)response.message[i] << 24) | ((uint32_t)response.message[i+1] << 16) | ((uint16_t)response.message[i+2] << 8) | response.message[i+3];
            i+= data_len;
            DF(OK("timestamp: %lu") " | ", timestamp);
            break;
          default:
            DF(NOK("unknown type 0x%02x") " | ", data_type);
            break;
        }
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
