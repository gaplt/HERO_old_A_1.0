#include "communicate_task.h"
#include "remote_control.h"
#include "INS_task.h"
#include "bsp_crc.h"
#include "gimbal_task.h"
#include "chassic_task.h"
#include "bsp_usart.h"
#include <string.h>
#include "referee.h"
#include "stm32f4xx_it.h"
int size_proto;

uint16_t shoot_logo = 0;


void communicate_task(void const * argument)
{
	

	size_proto = sizeof(hero_A_protocol_t);

	hero_A_protocol.frameHead = 0x5A;
	hero_A_protocol.frameTail = 0xA5;
	
	for(;;){
//		shoot_logo = HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0);
		if (ext_game_robot_status.mains_power_chassis_output == 0)
		{
    hero_A_protocol.hero_referee_data.cooling_heat_42mm = 0 ;		
		}
		else 
		{
		hero_A_protocol.hero_referee_data.cooling_heat_42mm = ext_game_robot_status.shooter_id1_42mm_cooling_rate;
		}

    hero_A_protocol.hero_referee_data.cooling_limit_42mm = ext_game_robot_status.shooter_id1_42mm_cooling_limit;
	  hero_A_protocol.hero_dial_logo_data.logo_shoot = shoot_logo ;
		hero_A_protocol.CRC16 = CRC16_calculate((uint8_t *)&hero_A_protocol, 7, 0xFFFF);		
		HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&hero_A_protocol, size_proto);
		
		osDelay(10);
		

	}
}