/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H2BR0_adc.h
 Description   : Header file provides configuration of the ADC instances.

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Variables ******************************************************/
extern ADC_HandleTypeDef hadc1;

/* Exported Functions ******************************************************/
extern void MX_ADC1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
