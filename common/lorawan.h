#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <avr/io.h>
#include "lorawan_struct.h"
#include "rfm95.h"
#include "pins.h"

class LORAWAN {
  public:
    LORAWAN();

    void     init(pins_t *cs = &pins_csrfm, pins_t *dio0 = &pins_dio0, pins_t *dio1 = &pins_dio1);
    void     set_otaa(uint8_t *deveui, uint8_t *appeui, uint8_t *appkey, const uint16_t counter = 0);
    void     set_abp(uint8_t *devaddr, uint8_t *nwkskey, uint8_t *appskey, const uint16_t counter = 0);
    Status   join(uint8_t wholescan=0);
    Status   send(const Packet *payload, Packet *rx_payload);
    Status   send(const Packet *payload, const uint8_t datarate = 0, Packet *rx_payload = 0);
    uint16_t calc_airtime(uint8_t len, uint8_t datarate);

    void     create_package(const Packet *payload, Packet *lora);
    void     cipher(Packet *payload, const uint16_t counter, const uint8_t direction, const uint8_t fport = 1, const uint8_t *devaddr = 0);

    uint16_t counter = 0;
    uint8_t appskey[16] = {0};
    uint8_t nwkskey[16] = {0};
    uint8_t devaddr[4] = {0};
    Lora_session session = { .nwkskey=nwkskey, .appskey=appskey, .devaddr=devaddr, .counter=counter };

  private:
    uint8_t deveui[8] = {0};
    uint8_t appeui[8] = {0};
    uint8_t appkey[16] = {0};
    uint16_t devnonce = 0;
    Lora_otaa otaa = { .deveui=deveui, .appeui=appeui, .appkey=appkey, .devnonce=devnonce };
    RFM95 rfm95;

    void     send_join_request(uint8_t channel, uint8_t datarate);
    Status   decode_data_down(Packet *payload);
    Status   decode_join_accept();
    uint8_t  is_same(uint8_t *arr1, uint8_t *arr2, uint8_t len);
};

#endif
