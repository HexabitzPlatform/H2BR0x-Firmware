/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H2BR0_adc.c
 Description   : Configures the ADC for the H2BR0 module to support EXG signal acquisition.
 ADC Configuration: Initializes ADC1 with 12-bit resolution, single-channel mode, and DMA for continuous data acquisition.
 GPIO Setup: Configures PA4 as the analog input pin for ADC1 channel 4.
 DMA Configuration: Sets up DMA2 Channel 1 for ADC data transfer in circular mode.
 De-initialization: Provides functions to safely disable ADC and associated GPIO and DMA resources.
*/

/* Includes ****************************************************************/
#include "H2BR0_adc.h"

/* Exported Variables ******************************************************/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/***************************************************************************/
/* Configure ADC ***********************************************************/
/***************************************************************************/
void EXG_ADC(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	HAL_ADC_Init(&hadc1);

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/***************************************************************************/
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (adcHandle->Instance == ADC1) {
		/* ADC1 clock enable */
		__HAL_RCC_ADC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/**ADC1 GPIO Configuration
		 PA4     ------> ADC1_IN4
		 */
		GPIO_InitStruct.Pin = ADC_INPUT_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ADC_INPUT_GPIO_PORT, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		/* ADC1 Init */
		hdma_adc1.Instance = DMA2_Channel1;
		hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc1.Init.Mode = DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
		HAL_DMA_Init(&hdma_adc1);

		__HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

	}
}

/***************************************************************************/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {

	if (adcHandle->Instance == ADC1) {
		/* Peripheral clock disable */
		__HAL_RCC_ADC_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		 PA4     ------> ADC1_IN4
		 */
		HAL_GPIO_DeInit(ADC_INPUT_GPIO_PORT, ADC_INPUT_PIN);

		/* ADC1 DMA DeInit */
		HAL_DMA_DeInit(adcHandle->DMA_Handle);

	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
