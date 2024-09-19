#ifndef __CRC_H
#define __CRC_H

#include "stdint.h"

//crc8
uint8_t crc8_get_checksum(uint8_t *pdata, uint16_t length, uint8_t crc8);
uint32_t crc8_verify_checksum(uint8_t *pdata, uint16_t length);
void crc8_set_checksum(uint8_t *pdata, uint16_t length);

//crc16
uint16_t crc16_get_checksum(uint8_t *pdata, uint32_t length, uint16_t crc16);
uint32_t crc16_verify_checksum(uint8_t *pdata, uint16_t length);
void crc16_set_checksum(uint8_t *pdata, uint16_t length);

//crc_ccitt
uint16_t crc_ccitt_get_checksum(uint8_t const *pdata, uint16_t length, uint16_t crc_init);

#endif
