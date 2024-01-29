/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H2BR0.h
 Description   : Header file for module H2BR0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H2BR0_H
#define H2BR0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H2BR0_MemoryMap.h"
#include "H2BR0_uart.h"
#include "H2BR0_gpio.h"
#include "H2BR0_dma.h"
#include "H2BR0_inputs.h"
#include "H2BR0_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H2BR0


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 0
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */
#define P1uart &huart6
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart6


/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6


/* Module-specific Definitions */

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_7

#define NUM_MODULE_PARAMS		1

/* EXG Module GPIO Pinout */
#define SDN_EXG_Pin             GPIO_PIN_6
#define SDN_EXG_GPIO_Port       GPIOA
#define LODP_EXG_Pin            GPIO_PIN_7
#define LODP_EXG_GPIO_Port      GPIOA
#define LODN_EXG_Pin            GPIO_PIN_0
#define LODN_EXG_GPIO_Port      GPIOB

/* EXG Module Special Timer */
#define EXG_TIM                 TIM2
#define EXG_TIM_PERIOD          TIM2->ARR
#define HANDLER_Timer_EXG       htim2

/* EXG Module Special ADC */
#define HANDLER_ADC_EXG         hadc1

/* EXG Module special parameters */
#define ADC_VREF                        3.3  //Volt
#define ADC_NUM_OF_STATES               4095
#define ECG_SAMPLE_TIME                 8333 //  micro sec	fs=120sps
#define EOG_SAMPLE_TIME                 10000 // micro sec	fs=100sps
#define EEG_SAMPLE_TIME                 10000 // micro sec fs=100sps
#define EMG_SAMPLE_TIME                 2000 //  micro sec fs=500sps
#define HEART_RATE_MIN                  40    // bpm
#define HEART_RATE_MAX                  120   // bpm
#define HEART_RATE_ARRAY_SIZE           5
#define EMG_MOVING_WINDOW               120   // samples
#define EMG_EVELOPE_GAIN_FACTOR         2.5   // samples
#define EMG_PULSE_MIN_THRESHOLD         0.045 // volt
#define EMG_PULSE_MAX_THRESHOLD         0.25  // volt
#define EMG_NOISY_PULSE_PERIOD_MS       50
#define EOG_BLINK_MAX_THRESHOLD         1.87  // volt
#define EOG_BLINK_MIN_THRESHOLD         1.52  // volt
#define EOG_NOISY_PULSE_PERIOD_MS       90
#define EOG_ONE_BLINK_PERIOD_MS         500
#define ECG_THRESHOLD                   0.25  //  volt
#define FILTER_TRANSIENT_STATE_SAMPLES  30
#define SHMITH_SHIFT                    0.03 // volt


/* Module EEPROM Variables */
// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* EXG Module_Status Type Definition */
typedef enum {
	H2BR0_OK =0,
	H2BR0_ERR_UnknownMessage,
	H2BR0_ERR_WrongParams,
	H2BR0_ERROR =255
} Module_Status;

typedef enum{
	EXG_ENABLED = 0,
	EXG_DISABLED,
}StatusType_EXG;

typedef enum{
	LeadP_CONNECTED_LeadN_CONNECTED        = 1,
	LeadP_CONNECTED_LeadN_NOTCONNECTED     = 1,
	LeadP_NOTCONNECTED_LeadN_CONNECTED     = 2,
	LeadP_NOTCONNECTED_LeadN_NOTCONNECTED  = 3,
}LeadsStatus_EXG;

typedef enum{
	ECG = 0,
	EOG,
	EEG,
	EMG,
}InputSignal_EXG;

typedef enum{
	NO_BLINK = 0,
	RIGHT_BLINK,
	LEFT_BLINK,
}EyeBlinkingStatus;

typedef struct{
	StatusType_EXG	EXGStatus;
	LeadsStatus_EXG statusOfLeads;
	InputSignal_EXG inputSignalType;
	uint32_t AdcValue;
	uint32_t sampleCounter;
	uint8_t  samplingFlag;
	float analogSample;
	float filteredSample;
	float tempFilterInputBuffer[5];
	float tempFilterOutputBuffer[5];
	float ECGBaselineFilteredSample;
	uint32_t HRCalculationLastTick;
	uint8_t heartRate;
	float heartRateArray[HEART_RATE_ARRAY_SIZE];
	uint8_t  heartRateIndex;
	uint8_t  heartRateLock;
	uint16_t previousHeartRate;
	float EMGRectifiedSample;
	float EMGEnvelopeSample;
	float movingWindowBuffer [EMG_MOVING_WINDOW];
	uint8_t windowBufferIndex;
	float sumOfSamplesValuesInWindow;
	uint8_t EMGPulseDetectionFlag;
	float EMGPulseDetectionThreshold;
	uint16_t EMGPulseDurationMsec;
	uint32_t EMGPulseRisingEdgeTick;
	uint8_t  EMGPulseDetectionLock;
	EyeBlinkingStatus eyeBlinkStatus;
	uint8_t  eyeBlinkDetectionFlag;
	uint16_t EOGPositivePulseDetectionTick;
	uint16_t EOGNegativePulseDetectionTick;
	uint8_t  EOGPositivePulseDetectionFlag;
	uint8_t  EOGNegativePulseDetectionFlag;
	uint8_t  EOGPositivePulseDetectionLock;
	uint8_t  EOGNegativePulseDetectionLock;
}EXG_t;

/* Export Module typedef structure */
extern EXG_t exg;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);

/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

Module_Status EXG_Init(EXG_t *EXGStruct ,InputSignal_EXG inputSignal);
Module_Status EXG_SignalProcessing(EXG_t *EXGStruct);
Module_Status ECG_Sample(EXG_t *EXGStruct, float *sample, float *filteredSample );
Module_Status EOG_Sample(EXG_t *EXGStruct, float *sample, float *filteredSample );
Module_Status EEG_Sample(EXG_t *EXGStruct, float *sample, float *filteredSample );
Module_Status EMG_Sample(EXG_t *EXGStruct, float *sample, float *filteredSample, float *rectifiedSample, float *envelopeSample);
Module_Status EMG_SetThreshold(EXG_t *EXGStruct, uint8_t threshold);
Module_Status EMG_CheckPulse(EXG_t *EXGStruct, uint8_t *EMGDetectionFlag, uint16_t *EMGDurationMsec);
Module_Status ECG_HeartRate(EXG_t *EXGStruct, uint8_t *heartRate);
Module_Status CheckEyeBlink(EXG_t *EXGStruct, EyeBlinkingStatus *eyeBlinkStatus); // EOG
Module_Status PlotToTerminal(EXG_t *EXGStruct, UART_HandleTypeDef *huart);
Module_Status LeadsStatus(EXG_t *EXGStruct, LeadsStatus_EXG *leadsStatus);
/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */


#endif /* H2BR0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
