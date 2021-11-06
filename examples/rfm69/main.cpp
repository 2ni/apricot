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

  while (1) {
    DF("sending %03u | ", counter);
    len = sprintf(msg, "%u", counter);
    // rfm69.send(GATEWAY, msg, len);
    // DL("");
    // if (rfm69.send(GATEWAY, msg, len, response)) {
    //   DF("response: '%s'\n", response);
    // }
    if (rfm69.send_retry(GATEWAY, msg, len, &response, 2)) {
      // 0x%06lX
      DF(OK("from 0x%06lX: '%s' (%idBm)"), response.from, response.message, response.rssi);
    } else {
      D(NOK("no response"));
    }
    DL("");
    rfm69.sleep();

    counter++;
    clock.sleep_for(8192);
  }
}
