/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H2BR0_timers.h
 Description   : Header file provides configuration of the timer instances.

 */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __H2BR0_TIMERS_H__
#define __H2BR0_TIMERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


extern TIM_HandleTypeDef htim2;


void MX_TIM2_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __H2BR0_TIMERS_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
