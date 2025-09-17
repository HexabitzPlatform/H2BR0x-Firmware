/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H2BR0_adc.h
 Description   : Header file for ADC configuration in the H2BR0 module.
 ADC Interface: Declares functions and variables for ADC initialization to support EXG signal acquisition.
 Exported Variables: Defines ADC1 handle for external use in EXG signal processing.
 Exported Functions: Provides interface for ADC configuration (EXG_ADC).
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
extern void EXG_ADC(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
