#include "bsp_visual.h"


Visual_Type_t Visual_Ture;

uint8_t Visual_Receive(uint8_t* Buf,uint32_t *Len,USBD_HandleTypeDef *pdev)
{
	uint8_t result = USBD_OK;
	uint32_t i;
	//HAL_GPIO_TogglePin(GPIOG,LED7_Pin);
	result = USBD_CDC_ReceivePacket(pdev);
	for(i=0;i<Len[0];i++)
	{
		Visual_Ture.RInit_data[i] = * Buf++;
	}
	Visual_Ture.RInit_data[i] = '\n';
  Visual_Ture.flag=1;
	Visual_Ture.whileflag=1;
  return result;
}

