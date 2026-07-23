#include "decoder.h"
#include "bsp_crc.h"
#include <string.h>
#include "remote_control.h"
#include "status.h"

uint8_t data_buffer[128];
uint8_t data_buffer_A[128];


uint32_t  time_logo ;

void decode_board_C(uint8_t* src, uint32_t len)
{
	//收到数据小于46字节退出
	if (len < 46) {
		return;
	}
	
	int pos = 0;
	while(*(src + pos) != 0x5A && (len - pos) >= 42) {
		pos ++;
	}
	
	if ((len - pos) < 46) {
		return;
	}
	
	uint16_t crc16 = *(uint16_t *)(src + pos + 43);
	
	if (crc16 != CRC16_calculate(src + pos, 43, 0xFFFF) || *(src + pos + 45) != 0xA5) 
	{
		return;
	}
	
	memcpy(&hero_protocol, (src + pos), 46);
  
	time_logo = HAL_GetTick();
	
}

void decode_board_A(uint8_t* src, uint32_t len)
{
	//收到数据小于10字节退出
	if (len < 10) {
		return;
	}
	
	int pos = 0;
	while(*(src + pos) != 0x5A && (len - pos) >= 10) {
		pos ++;
	}
	
	if ((len - pos) < 10) {
		return;
	}
	
	uint16_t crc16 = *(uint16_t *)(src + pos + 7);
	
	if (crc16 != CRC16_calculate(src + pos, 7, 0xFFFF) || *(src + pos + 9) != 0xA5) 
	{
		return;
	}
	
	memcpy(&hero_A_protocol, (src + pos), 10);	
}