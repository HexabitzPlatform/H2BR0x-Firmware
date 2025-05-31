/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H2BR0.h
 Description   : Header file for module H2BR0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>> Interrupt mode timer (in which the signal specified by the function is processed EXG_SignalProcessing).
>> ADC to read the analog signal of the signals.
>>

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H2BR0_H
#define H2BR0_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H2BR0_MemoryMap.h"
#include "H2BR0_uart.h"
#include "H2BR0_gpio.h"
#include "H2BR0_dma.h"
#include "H2BR0_adc.h"
#include "H2BR0_inputs.h"
#include "H2BR0_eeprom.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H2BR0

/* Port-related Definitions */
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available Ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART5
#define _USART6

/* Port-UART Mapping */
#define UART_P1 &huart6
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart5

/* Module-specific Hardware Definitions ************************************/
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

/* GPIO Pin Definition */
#define SDN_EXG_PIN             GPIO_PIN_6
#define SDN_EXG_GPIO_PORT       GPIOA
#define LODP_EXG_PIN            GPIO_PIN_7
#define LODP_EXG_GPIO_PORT      GPIOA
#define LODN_EXG_PIN            GPIO_PIN_0
#define LODN_EXG_GPIO_PORT      GPIOB

/* ADC Pin Definition */
#define ADC_INPUT_PIN           GPIO_PIN_4
#define ADC_INPUT_GPIO_PORT     GPIOA

#define HANDLER_ADC_EXG         hadc1

/* Timer Definition */
#define EXG_TIM                 TIM2
#define EXG_TIM_PERIOD          TIM2->ARR
#define HANDLER_Timer_EXG       htim2

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_7

/* Module-specific Macro Definitions ***************************************/
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
#define MIN_PERIOD_MS		     100
#define MAX_TIMEOUT_MS		     0xFFFFFFFF


#define STREAM_TO_PORT                  1
#define STREAM_TO_Terminal              2
#define MIN_PERIOD_MS			    	100

#define NUM_MODULE_PARAMS		        10

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H2BR0_OK =0,
	H2BR0_ERR_LEADS_NOTCONNECTED,
	H2BR0_ERR_TERMINATED,
	H2BR0_ERR_WRONGPARAMS,
	H2BR0_ERR_UNKNOWNMESSAGE,
	H2BR0_ERROR =255
} Module_Status;

typedef enum{
	EXG_ENABLED = 0,
	EXG_DISABLED,
}StatusType_EXG;

typedef enum{
	LEADP_CONNECTED_LEADN_CONNECTED        = 0,
	LEADP_CONNECTED_LEADN_NOTCONNECTED     = 1,
	LEADP_NOTCONNECTED_LEADN_CONNECTED     = 2,
	LEADP_NOTCONNECTED_LEADN_NOTCONNECTED  = 3,
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
	uint8_t heartRate;
	uint8_t samplingFlag;
	uint8_t heartRateLock;
	uint8_t heartRateIndex;
	uint8_t windowBufferIndex;
	uint8_t EMGPulseDetectionFlag;
	uint8_t EMGPulseDetectionLock;
	uint8_t eyeBlinkDetectionFlag;
	uint8_t EOGPositivePulseDetectionFlag;
	uint8_t EOGNegativePulseDetectionFlag;
	uint8_t EOGPositivePulseDetectionLock;
	uint8_t EOGNegativePulseDetectionLock;

	uint16_t previousHeartRate;
	uint16_t EMGPulseDurationMsec;
	uint16_t EOGPositivePulseDetectionTick;
	uint16_t EOGNegativePulseDetectionTick;

	uint32_t EMGPulseRisingEdgeTick;
	uint32_t AdcValue;
	uint32_t sampleCounter;
	uint32_t HRCalculationLastTick;

	float analogSample;
	float filteredSample;
	float EMGEnvelopeSample;
	float EMGRectifiedSample;
	float tempFilterInputBuffer[5];
	float tempFilterOutputBuffer[5];
	float ECGBaselineFilteredSample;
	float sumOfSamplesValuesInWindow;
	float EMGPulseDetectionThreshold;
	float movingWindowBuffer [EMG_MOVING_WINDOW];
	float heartRateArray[HEART_RATE_ARRAY_SIZE];

	StatusType_EXG	EXGStatus;
	LeadsStatus_EXG statusOfLeads;
	InputSignal_EXG inputSignalType;
	EyeBlinkingStatus eyeBlinkStatus;

}EXG_t;

/* Exported UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status EXG_Init(InputSignal_EXG inputSignal);
Module_Status ECG_Sample(float *sample, float *filteredSample);
Module_Status EOG_Sample(float *sample, float *filteredSample);
Module_Status EEG_Sample(float *sample, float *filteredSample);
Module_Status EMG_Sample(float *sample, float *filteredSample, float *rectifiedSample, float *envelopeSample);
Module_Status EMG_SetThreshold(uint8_t threshold);
Module_Status EMG_CheckPulse(uint8_t *EMGDetectionFlag, uint16_t *EMGDurationMsec);
Module_Status ECG_HeartRate(uint8_t *heartRate);
Module_Status EOG_CheckEyeBlink(EyeBlinkingStatus *eyeBlinkStatus);
Module_Status PlotToTerminal(uint8_t port);
Module_Status LeadsStatus(LeadsStatus_EXG *leadsStatus);
Module_Status SampletoPort(uint8_t module,uint8_t port, InputSignal_EXG inputSignal);
Module_Status StreamtoPort(uint8_t module,uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout);
Module_Status StreamToTerminal(uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout);
Module_Status StreamToBuffer(float *buffer, InputSignal_EXG function, uint32_t Numofsamples, uint32_t timeout);
#endif /* H2BR0_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
