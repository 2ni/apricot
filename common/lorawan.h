#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <avr/io.h>
#include "lorawan_struct.h"
#include "rfm95.h"
#include "pins.h"

#define LORA_MAC_LINKCHECK  0x02
#define LORA_MAC_LINKADR    0x03
#define LORA_MAC_DUTYCYCLE  0x04
#define LORA_MAC_RXPARAM    0x05
#define LORA_MAC_DEVSTATUS  0x06
#define LORA_MAC_NEWCHANNEL 0x07
#define LORA_MAC_RXTIMING   0x08
#define LORA_MAC_TXPARAM    0x09
#define LORA_MAC_DICHANNEL  0x0A
#define LORA_MAC_DEVICETIME 0x0D

#define TICKS_PER_SEC 4096 // see also common/clock.cpp

class LORAWAN {
  public:
    LORAWAN(uint8_t adaptive = 1, uint8_t check_link_nth = 0);

    // void     init(pins_t *cs = &pins_csrfm, pins_t *dio0 = &pins_dio0, pins_t *dio1 = &pins_dio1);
    uint8_t  init(pins_t *cs = &pins_csrfm);
    uint8_t  init(Lora_session *ee_session);
    uint8_t  init(pins_t *cs, Lora_session *i_ee_session);
    uint8_t  has_session();
    void     reset_session();
    void     set_otaa(uint8_t *deveui, uint8_t *appeui, uint8_t *appkey);
    void     set_abp(uint8_t *devaddr, uint8_t *nwkskey, uint8_t *appskey, const uint16_t counter = 0);
    Status   join(uint8_t wholescan=0);
    Status   send(const Packet *payload, Packet *rx_payload, uint8_t ack = 0);
    Status   send(const Packet *payload, Packet *rx_payload, const uint8_t datarate, uint8_t ack);
    void     process_cmds(Packet *cmds);
    uint16_t calc_airtime(uint8_t len, uint8_t datarate);

    void     create_package(const Packet *payload, Packet *lora, uint8_t ack = 0, uint8_t adrack = 0, uint8_t *cmds = 0);
    void     cipher(Packet *payload, const uint16_t counter, const uint8_t direction, const uint8_t fport = 1, const uint8_t *devaddr = 0);

    uint8_t  get_next_frq_pos();

    Lora_session session;
    Lora_session *ee_session;
    RFM95 rfm95;

    uint8_t  queue_cmd_tx[8] = {0};
    uint8_t  queue_p = 0;

  private:
    Lora_otaa otaa = { .deveui={0}, .appeui={0}, .appkey={0}, .devnonce=0 };
    void     _set_persistent(Lora_session *ee_session);
    void     _persist_session();

    uint8_t  check_link_nth;
    uint8_t  adaptive; // if set ADR = adaptive datarate done by gateway
    uint8_t  persistent = 0;  // we can't test it pointer to EEMEM variable is set so we use a variable
    uint8_t  chmask_current_pos; // points to the current chmask bit used to select the frequency

    Status   decode_data_down(Packet *payload, Packet *rx_mac, uint8_t ack_requested = 0);
    Status   decode_join_accept();
    uint8_t  is_same(uint8_t *arr1, uint8_t *arr2, uint8_t len);
};

#endif
