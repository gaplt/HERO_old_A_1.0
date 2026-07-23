#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "main.h"
#include "tim.h"

#define BUZZER_ON() \
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 60)

#define BUZZER_OFF() \
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0)

#define BUZZER_SET(__var__) \
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, __var__)

#endif //__BSP_BUZZER_H