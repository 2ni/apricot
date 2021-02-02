#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <avr/io.h>
#include "lorawan_struct.h"
#include "rfm95.h"
#include "pins.h"

#define LORA_MAC_LINKCHECK  0x02
#define LORA_MAC_LINKADR    0x03
#define LORA_MAC_CYCLE      0x04
#define LORA_MAC_RXPARAM    0x05
#define LORA_MAC_DEVSTATUS  0x06
#define LORA_MAC_NEWCHANNEL 0x07
#define LORA_MAC_RXTIMING   0x08
#define LORA_MAC_TXPARAM    0x09
#define LORA_MAC_DICHANNEL  0x0A

class LORAWAN {
  public:
    LORAWAN();
    LORAWAN(uint8_t adr, uint8_t check_link_nth);

    // void     init(pins_t *cs = &pins_csrfm, pins_t *dio0 = &pins_dio0, pins_t *dio1 = &pins_dio1);
    void     init(pins_t *cs = &pins_csrfm);
    void     set_otaa(uint8_t *deveui, uint8_t *appeui, uint8_t *appkey);
    void     set_abp(uint8_t *devaddr, uint8_t *nwkskey, uint8_t *appskey, const uint16_t counter = 0);
    Status   join(uint8_t wholescan=0);
    Status   send(const Packet *payload, Packet *rx_payload, uint8_t ack = 0);
    Status   send(const Packet *payload, Packet *rx_payload, const uint8_t datarate, uint8_t ack);
    void     process_cmds(Packet *cmds);
    uint16_t calc_airtime(uint8_t len, uint8_t datarate);

    void     create_package(const Packet *payload, Packet *lora, uint8_t ack = 0, uint8_t adrack = 0, uint8_t *cmds = 0);
    void     cipher(Packet *payload, const uint16_t counter, const uint8_t direction, const uint8_t fport = 1, const uint8_t *devaddr = 0);

    Lora_session session;

  private:
    Lora_otaa otaa = { .deveui={0}, .appeui={0}, .appkey={0}, .devnonce=0 };
    uint8_t  check_link_nth;
    uint8_t  adr;
    RFM95 rfm95;
    uint8_t  queue_cmd_tx[8];
    uint8_t  queue_p = 0;

    void     send_join_request(uint8_t channel, uint8_t datarate);
    Status   decode_data_down(Packet *payload, Packet *rx_mac, uint8_t ack_requested = 0);
    Status   decode_join_accept();
    uint8_t  is_same(uint8_t *arr1, uint8_t *arr2, uint8_t len);
};

#endif
