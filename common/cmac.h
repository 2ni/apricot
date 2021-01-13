#ifndef __CMAC_H__
#define __CMAC_H__

#include <avr/io.h>
#include "lorawan_struct.h"

void aes128_b0(const Packet *packet, uint16_t const counter, const uint8_t direction, const uint8_t *dev_addr, Packet *b0);
void aes128_generate_subkeys(const uint8_t *key, uint8_t *key1, uint8_t *key2);
void aes128_mic(const uint8_t *key, const Packet *packet, Packet *mic);

#endif
