#ifndef BRIDGE_HPP_
#define BRIDGE_HPP_
#include "rmserial.h"

extern uint16_t crc_tab16[256];

void Init_CRC16();

uint16_t CRC16_calc(uint8_t* data, uint32_t num_bytes);

#endif