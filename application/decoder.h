#ifndef __DECODER_H
#define __DECODER_H

#include <stdint.h>

#include "protocol.h"

void decode_board_C(uint8_t* src, uint32_t len);

extern uint8_t data_buffer[128];
extern uint32_t  time_logo ;
#endif //__DECODER_H
