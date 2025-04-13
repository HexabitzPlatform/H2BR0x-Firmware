/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H2BR0_timers.c
 Description   : Peripheral timers setup source file.

 Required MCU resources :

 >> Timer 16 for micro-sec delay.
 >> Timer 17 for milli-sec delay.

 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Functions ******************************************************/
void TIM_USEC_Init(void);
void TIM_MSEC_Init(void);
void MX_IWDG_Init(void);

extern void MX_TIM2_Init(void); /* EXG special timer */

/* Exported Variables ******************************************************/
TIM_HandleTypeDef htim16; /* micro-second delay counter */
TIM_HandleTypeDef htim17; /* milli-second delay counter */
IWDG_HandleTypeDef hiwdg;

extern TIM_HandleTypeDef htim2;  /* EXG special timer */

/***************************************************************************/
/* Configure Timers ********************************************************/
/***************************************************************************/
/* IWDG init function */
void MX_IWDG_Init(void) {

	/* Reload Value = [(Time * 32 KHz) / (4 * 2^(pr) * 1000)] - 1
	 * RL = [(500 mS * 32000) / (4 * 2^1 * 1000)]  - 1 = 2000 - 1 = 1999
	 * timeout time = 500 mS
	 * Pre-scaler = 8
	 * Reload Value = 1999
	 *  */

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 1999;

	HAL_IWDG_Init(&hiwdg);

}

/***************************************************************************/
/* EXG special timer */
void MX_TIM2_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 64 - 1; /* Prescaler value: (Prescaler + 1) = 64 */
	/* Timer clock frequency = SYSCLK / (Prescaler + 1) */
	/* Timer clock = 64MHz / 64 = 1MHz */

	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

	/* Initial period value: 2000 */
	/* The actual period is defined by EXG_TIM_PERIOD later */
	htim2.Init.Period = 2000;

	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	/* Using the internal clock source (SYSCLK after prescaler) */

	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/***************************************************************************/
/* Micro-seconds timebase init function - TIM16 (16-bit) */
void TIM_USEC_Init(void) {
	TIM_MasterConfigTypeDef sMasterConfig;

	/* Peripheral clock enable */
	__TIM16_CLK_ENABLE();

	/* Peripheral configuration */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 47;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 0XFFFF;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim16);

	HAL_TIM_Base_Start(&htim16);

}

/***************************************************************************/
/* Milli-seconds timebase init function - TIM17 (16-bit) */
void TIM_MSEC_Init(void) {
	TIM_MasterConfigTypeDef sMasterConfig;

	/* Peripheral clock enable */
	__TIM17_CLK_ENABLE();

	/* Peripheral configuration */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 47999;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 0xFFFF;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim17);

	HAL_TIM_Base_Start(&htim17);
}

/***************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle) {

	if (tim_baseHandle->Instance == TIM2) {
		/* TIM2 clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		/* TIM2 interrupt Init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);

	}
}

/***************************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle) {

	if (tim_baseHandle->Instance == TIM2) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* TIM2 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM2_IRQn);

	}
}

/***************************************************************************/
/* Load and start micro-second delay counter */
void StartMicroDelay(uint16_t Delay) {
	uint32_t t0 = 0;

	portENTER_CRITICAL();

	if (Delay) {
		t0 = htim16.Instance->CNT;

		while (htim16.Instance->CNT - t0 <= Delay) {
		};
	}

	portEXIT_CRITICAL();
}

/***************************************************************************/
/* Load and start milli-second delay counter */
void StartMilliDelay(uint16_t Delay) {
	uint32_t t0 = 0;

	portENTER_CRITICAL();

	if (Delay) {
		t0 = htim17.Instance->CNT;

		while (htim17.Instance->CNT - t0 <= Delay) {
		};
	}

	portEXIT_CRITICAL();
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
