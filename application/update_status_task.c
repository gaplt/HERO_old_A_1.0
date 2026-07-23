#include "update_status_task.h"
#include <string.h>
#include "decoder.h"
#include "bsp_buzzer.h"



void statusUpdateTask(void const * argument)
{
  /* USER CODE BEGIN update_status_task */
  /* Infinite loop */
	
	
	
	
	
	for(;;)
	{
		
		
		//·äÃùÆ÷¼ì²âACÍ¨ÐÅµÄ×´Ì¬		
	
//		uint32_t cur = HAL_GetTick();
//		
//		//¼ì²âÓëC°åÍ¨ÐÅ×´Ì¬
//		if(cur - time_logo > 200)
//		{
//		for(;;)
//     {
//			 BUZZER_ON();
//		   HAL_Delay(500);
//			 if(cur - time_logo < 200||cur - time_logo == 200)
//			 {BUZZER_OFF();}
//				 break;
//		 }
//		} 
//	else 
//		{
//		 BUZZER_OFF();
//		}



		
		osDelay(10);
	}
  /* USER CODE END update_status_task */
}
