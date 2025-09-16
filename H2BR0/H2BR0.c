/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H2BR0.c
 Description   : Source code for module H2BR0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H2BR0_inputs.h"

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart6;
TIM_HandleTypeDef htim2;  /* EXG special timer */
TaskHandle_t EXGTaskHandle = NULL;
TaskHandle_t EXGSignalProcessingHandle = NULL;
EXG_t exg;
/* Private Variables *******************************************************/
bool plotEnabled  =false ;
uint8_t plotPort  ;
/* Module Parameters */
uint16_t ECG_Index = 0;
uint16_t EOG_Index = 0;
uint16_t EEG_Index = 0;
uint16_t EMG_Index = 0;
uint32_t end_time, start_time;
float Loop_time;
/* Module exported parameters ------------------------------------------------*/
/* Exported Typedef */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = {};
/* Private Function Prototypes *********************************************/
void MX_TIM2_Init(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
uint8_t ClearROtopology(void);
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift);

/* Local Function Prototypes ***********************************************/
//void EXGTask(void *argument);
void EXGSignalProcessing(void *argument);
void EXG_Enable();
void EXG_Disable();
void EXG_Reset();
void GetSamplingFlag(uint8_t *samplingFlag);
void ResetSamplingFlag();
void SetSamplingFlag();
void ECG_Filter();
void ECG_BaselineFilter();
void ECG_HeartRateCalculation();
void EOG_Filter();
void EEG_Filter();
void EMG_Filter();
void EyeBlinkDetection();
void EMG_Rectifying();
void EMG_EnvelopeDetection();
void CheckLeadsStatus(LeadsStatus_EXG *leadsStatus);
Module_Status EXG_SignalProcessing(void);
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule);
/* Local Typedef related to stream functions */

/* Create CLI commands *****************************************************/

///* CLI command structure ***************************************************/


/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H2BR0 module initialization */
void Module_Peripheral_Init(void) {

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();

	MX_TIM2_Init();
	MX_ADC1_Init();

	//Circulating DMA Channels ON All Module
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		}else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		}
	}

	xTaskCreate(EXGSignalProcessing, (const char*) "EXGSignalProcessingTask",
			configMINIMAL_STACK_SIZE, NULL, osPriorityRealtime - osPriorityIdle,
			&EXGSignalProcessingHandle);

}

/***************************************************************************/
/* H2BR0 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src,
		uint8_t dst, uint8_t shift) {
	Module_Status result = H2BR0_OK;

	switch (code) {

	default:
		result = H2BR0_ERR_UNKNOWNMESSAGE;
		break;
	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART3)
		return P1;
	else if(huart->Instance == USART1)
		return P2;
	else if(huart->Instance == USART4)
		return P3;
	else if(huart->Instance == USART2)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	
	return 0;
}


/***************************************************************************/
/* This function is useful only for input (sensors) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
	Module_Status status = BOS_OK;

	return status;
}
/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {

}
/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
void EXGSignalProcessing(void *argument) {

	for (;;) {
		// Wait for ISR notification
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Process EXG Signal
		EXG_SignalProcessing();
	}

}

/***************************************************************************/
void EXG_Enable() {
	HAL_GPIO_WritePin(SDN_EXG_GPIO_PORT, SDN_EXG_PIN, GPIO_PIN_SET);
	exg.EXGStatus = EXG_ENABLED;
}

/***************************************************************************/
void EXG_Disable() {
	HAL_GPIO_WritePin(SDN_EXG_GPIO_PORT, SDN_EXG_PIN, GPIO_PIN_RESET);
	exg.EXGStatus = EXG_DISABLED;
}

/***************************************************************************/
void EXG_Reset() {
	EXG_Disable();
	HAL_Delay(10);
	EXG_Enable();
}

/***************************************************************************/
void GetSamplingFlag(uint8_t *samplingFlag) {
	*samplingFlag = exg.samplingFlag;
}

/***************************************************************************/
void ResetSamplingFlag() {
	exg.samplingFlag = 0;
}

/***************************************************************************/
void SetSamplingFlag() {
	exg.samplingFlag = 1;
}

/***************************************************************************/
void CheckLeadsStatus(LeadsStatus_EXG *leadsStatus) {
	GPIO_PinState LODPStatus;
	GPIO_PinState LODNStatus;

	LODPStatus = HAL_GPIO_ReadPin(LODP_EXG_GPIO_PORT, LODP_EXG_PIN);
	LODNStatus = HAL_GPIO_ReadPin(LODN_EXG_GPIO_PORT, LODN_EXG_PIN);

	if (LODPStatus == GPIO_PIN_RESET && LODNStatus == GPIO_PIN_RESET)
		*leadsStatus = LEADP_CONNECTED_LEADN_CONNECTED;
	else if (LODPStatus == GPIO_PIN_RESET && LODNStatus == GPIO_PIN_SET)
		*leadsStatus = LEADP_CONNECTED_LEADN_NOTCONNECTED;
	else if (LODPStatus == GPIO_PIN_SET && LODNStatus == GPIO_PIN_RESET)
		*leadsStatus = LEADP_NOTCONNECTED_LEADN_CONNECTED;
	else
		*leadsStatus = LEADP_NOTCONNECTED_LEADN_NOTCONNECTED;

	exg.statusOfLeads = *leadsStatus;
}

/***************************************************************************/
void ECG_Filter() {
	float input1 = exg.analogSample;
	float preInput1 = exg.tempFilterInputBuffer[0];
	float beforePreInput1 = exg.tempFilterInputBuffer[1];
	float output1;
	float preOutput1 = exg.tempFilterOutputBuffer[0];
	float beforePreOutput1 = exg.tempFilterOutputBuffer[1];

	// LPF: Fs=120sps, Fc=40Hz, Order=2
	output1 = -0.6202 * preOutput1 - 0.2404 * beforePreOutput1 + 0.4652 * input1
			+ 0.9303 * preInput1 + 0.4652 * beforePreInput1;
	beforePreInput1 = preInput1;
	preInput1 = input1;
	beforePreOutput1 = preOutput1;
	preOutput1 = output1;

	exg.tempFilterInputBuffer[0] = preInput1;
	exg.tempFilterInputBuffer[1] = beforePreInput1;
	exg.tempFilterOutputBuffer[0] = preOutput1;
	exg.tempFilterOutputBuffer[1] = beforePreOutput1;

	float input2 = output1;
	float preInput2 = exg.tempFilterInputBuffer[2];
	float beforePreInput2 = exg.tempFilterInputBuffer[3];
	float output2;
	float preOutput2 = exg.tempFilterOutputBuffer[2];
	float beforePreOutput2 = exg.tempFilterOutputBuffer[3];

	// LPF: Fs=120sps, Fc=40Hz, Order=2
	output2 = -0.6202 * preOutput2 - 0.2404 * beforePreOutput2 + 0.4652 * input2
			+ 0.9303 * preInput2 + 0.4652 * beforePreInput2;
	beforePreInput2 = preInput2;
	preInput2 = input2;
	beforePreOutput2 = preOutput2;
	preOutput2 = output2;

	exg.tempFilterInputBuffer[2] = preInput2;
	exg.tempFilterInputBuffer[3] = beforePreInput2;
	exg.tempFilterOutputBuffer[2] = preOutput2;
	exg.tempFilterOutputBuffer[3] = beforePreOutput2;
	exg.filteredSample = output2;
}

/***************************************************************************/
void EOG_Filter() {
	float input = exg.analogSample;
	float preInput = exg.tempFilterInputBuffer[0];
	float beforePreInput = exg.tempFilterInputBuffer[1];
	float output;
	float preOutput = exg.tempFilterOutputBuffer[0];
	float beforePreOutput = exg.tempFilterOutputBuffer[1];

	// LPF: Fs=100sps, Fc=25Hz, Order=2
	output = -0.0 * preOutput - 0.1716 * beforePreOutput + 0.2929 * input
			+ 0.5858 * preInput + 0.2929 * beforePreInput;
	beforePreInput = preInput;
	preInput = input;
	beforePreOutput = preOutput;
	preOutput = output;

	exg.tempFilterInputBuffer[0] = preInput;
	exg.tempFilterInputBuffer[1] = beforePreInput;
	exg.tempFilterOutputBuffer[0] = preOutput;
	exg.tempFilterOutputBuffer[1] = beforePreOutput;
	exg.filteredSample = output;
}

/***************************************************************************/
void EEG_Filter() {
	float input = exg.analogSample;
	float preInput = exg.tempFilterInputBuffer[0];
	float beforePreInput = exg.tempFilterInputBuffer[1];
	float output;
	float preOutput = exg.tempFilterOutputBuffer[0];
	float beforePreOutput = exg.tempFilterOutputBuffer[1];

	// LPF: Fs=100sps, Fc=30Hz, Order=2
	output = -0.3695 * preOutput - 0.1958 * beforePreOutput + 0.3913 * input
			+ 0.7827 * preInput + 0.3913 * beforePreInput;
	beforePreInput = preInput;
	preInput = input;
	beforePreOutput = preOutput;
	preOutput = output;

	exg.tempFilterInputBuffer[0] = preInput;
	exg.tempFilterInputBuffer[1] = beforePreInput;
	exg.tempFilterOutputBuffer[0] = preOutput;
	exg.tempFilterOutputBuffer[1] = beforePreOutput;
	exg.filteredSample = output;
}

/***************************************************************************/
void EMG_Filter() {
	float input1 = exg.analogSample;
	float preInput1 = exg.tempFilterInputBuffer[0];
	float beforePreInput1 = exg.tempFilterInputBuffer[1];
	float output1;
	float preOutput1 = exg.tempFilterOutputBuffer[0];
	float beforePreOutput1 = exg.tempFilterOutputBuffer[1];

	// LPF: Fs=500sps, Fc=150Hz, Order=2
	output1 = -0.3695 * preOutput1 - 0.1958 * beforePreOutput1 + 0.3913 * input1
			+ 0.7827 * preInput1 + 0.3913 * beforePreInput1;
	beforePreInput1 = preInput1;
	preInput1 = input1;
	beforePreOutput1 = preOutput1;
	preOutput1 = output1;

	exg.tempFilterInputBuffer[0] = preInput1;
	exg.tempFilterInputBuffer[1] = beforePreInput1;
	exg.tempFilterOutputBuffer[0] = preOutput1;
	exg.tempFilterOutputBuffer[1] = beforePreOutput1;

	float input2 = output1;
	float preInput2 = exg.tempFilterInputBuffer[2];
	float beforePreInput2 = exg.tempFilterInputBuffer[3];
	float output2;
	float preOutput2 = exg.tempFilterOutputBuffer[2];
	float beforePreOutput2 = exg.tempFilterOutputBuffer[3];

	// HPF: Fs=500sps, Fc=20Hz, Order=2
	output2 = 1.6475 * preOutput2 - 0.7009 * beforePreOutput2 + 0.8371 * input2
			- 1.6742 * preInput2 + 0.8371 * beforePreInput2;
	beforePreInput2 = preInput2;
	preInput2 = input2;
	beforePreOutput2 = preOutput2;
	preOutput2 = output2;
	exg.tempFilterInputBuffer[2] = preInput2;
	exg.tempFilterInputBuffer[3] = beforePreInput2;
	exg.tempFilterOutputBuffer[2] = preOutput2;
	exg.tempFilterOutputBuffer[3] = beforePreOutput2;
	exg.filteredSample = output2;
}

/***************************************************************************/
void ECG_BaselineFilter() {
	float input = exg.filteredSample;
	float preInput = exg.tempFilterInputBuffer[4];
	float output;
	float preOutput = exg.tempFilterOutputBuffer[4];

	// HPF: Fs=120sps, Fc=7Hz, Order=1
	output = 0.6873 * preOutput + 0.8436 * input - 0.8436 * preInput;
	preInput = input;
	preOutput = output;
	exg.tempFilterInputBuffer[4] = preInput;
	exg.tempFilterOutputBuffer[4] = preOutput;
	exg.ECGBaselineFilteredSample = output;
}

/***************************************************************************/
void ECG_HeartRateCalculation() {
	uint16_t period;
	float HR;
	float input = exg.ECGBaselineFilteredSample;
	float heartRateSum = 0;

	if (input >= ECG_THRESHOLD && exg.heartRateLock == 0) {
		exg.heartRateLock = 1;
		period = HAL_GetTick() - exg.HRCalculationLastTick; // find time between tow beats in msec.
		HR = 60000.0 / (float) period;
		if (HR >= HEART_RATE_MIN && HR <= HEART_RATE_MAX) {
			// if relative change between current HR and old HR within specific range, send HR else ignore sending current value (there is noise)
			if ((HR >= 0.8 * exg.previousHeartRate)
					&& (HR <= 1.2 * exg.previousHeartRate)) {
				exg.heartRateArray[exg.heartRateIndex++] = HR;
				if (exg.heartRateIndex == HEART_RATE_ARRAY_SIZE) {
					exg.heartRateIndex = 0;
					for (uint8_t i = 0; i < HEART_RATE_ARRAY_SIZE; i++)
						heartRateSum += exg.heartRateArray[i];
					exg.heartRate = roundf(
							heartRateSum / HEART_RATE_ARRAY_SIZE);
				}
			}
		} else {
			HR = 0;
			exg.heartRate = HR;
			exg.heartRateIndex = 0; // empty heartRateArray when happening wrong heart rate
		}
		exg.previousHeartRate = HR;
		exg.HRCalculationLastTick = HAL_GetTick();
	} else if (input < ECG_THRESHOLD)
		exg.heartRateLock = 0;
}

/***************************************************************************/
void EOG_EnvelopeDetection() {
	float input = exg.EMGRectifiedSample;
	uint8_t index = exg.windowBufferIndex;
	float lastSampleInWindow = exg.movingWindowBuffer[index];
	float sum = exg.sumOfSamplesValuesInWindow;
	float movingMean;

	sum = sum - lastSampleInWindow;
	sum = sum + input;  // new sample get into window
	exg.movingWindowBuffer[index] = input;
	index++;
	if (index == EMG_MOVING_WINDOW) // circular buffer
		index = 0;
	movingMean = (sum / EMG_MOVING_WINDOW) * EMG_EVELOPE_GAIN_FACTOR;

	exg.sumOfSamplesValuesInWindow = sum;
	exg.EMGEnvelopeSample = movingMean;
	exg.windowBufferIndex = index;
}

/***************************************************************************/
void EMG_Rectifying() {
	float input = exg.filteredSample;
	float absInput = input;

	if (input < 0.0)
		absInput = -input;
	exg.EMGRectifiedSample = absInput;
}

/***************************************************************************/
void EMG_EnvelopeDetection() {
	float input = exg.EMGRectifiedSample;
	uint8_t index = exg.windowBufferIndex;
	float lastSampleInWindow = exg.movingWindowBuffer[index];
	float sum = exg.sumOfSamplesValuesInWindow;
	float movingMean;

	sum = sum - lastSampleInWindow;
	sum = sum + input;  // new sample get into window
	exg.movingWindowBuffer[index] = input;
	index++;
	if (index == EMG_MOVING_WINDOW) // circular buffer
		index = 0;
	movingMean = (sum / EMG_MOVING_WINDOW) * EMG_EVELOPE_GAIN_FACTOR;

	exg.sumOfSamplesValuesInWindow = sum;
	exg.EMGEnvelopeSample = movingMean;
	exg.windowBufferIndex = index;
}

/***************************************************************************/
void EMG_PulseDetection() {
	float input = exg.EMGEnvelopeSample;
	float EMGPulseThreshold = exg.EMGPulseDetectionThreshold;

	/* detecting rising edge of pulse */
	if (input >= EMGPulseThreshold && exg.EMGPulseDetectionLock == 0) {
		exg.EMGPulseRisingEdgeTick = HAL_GetTick();
		exg.EMGPulseDetectionLock = 1;
	}
	/* detecting falling edge of pulse */
	else if (input < EMGPulseThreshold && exg.EMGPulseDetectionLock == 1) {

		uint16_t EMGPulseTime = HAL_GetTick() - exg.EMGPulseRisingEdgeTick;
		if (EMGPulseTime > EMG_NOISY_PULSE_PERIOD_MS) {
			exg.EMGPulseDurationMsec = EMGPulseTime;
			exg.EMGPulseDetectionFlag = 1;
		}
		exg.EMGPulseDetectionLock = 0;
	}
}

/***************************************************************************/
void EyeBlinkDetection() {
	float input = exg.filteredSample;

	/* detecting rising edge (start) of positive pulse */
	if (input >= EOG_BLINK_MAX_THRESHOLD
			&& exg.EOGPositivePulseDetectionLock == 0)
		exg.EOGPositivePulseDetectionLock = 1;

	/* detecting falling edge (finish) of positive pulse */
	else if (input < (EOG_BLINK_MAX_THRESHOLD - SHMITH_SHIFT)
			&& exg.EOGPositivePulseDetectionLock == 1) {
		exg.EOGPositivePulseDetectionLock = 0;
		exg.EOGPositivePulseDetectionTick = HAL_GetTick();
		exg.EOGPositivePulseDetectionFlag = 1;

		if (exg.EOGNegativePulseDetectionFlag == 1) {
			if (exg.EOGPositivePulseDetectionTick
					- exg.EOGNegativePulseDetectionTick< EOG_ONE_BLINK_PERIOD_MS) {
				exg.eyeBlinkStatus = LEFT_BLINK;
				exg.EOGNegativePulseDetectionFlag = 0;
				exg.EOGPositivePulseDetectionFlag = 0;
			} else
				exg.EOGNegativePulseDetectionFlag = 0;
		}
	} /* detecting falling edge (start) of negative pulse */
	else if (input <= EOG_BLINK_MIN_THRESHOLD
			&& exg.EOGNegativePulseDetectionLock == 0)
		exg.EOGNegativePulseDetectionLock = 1;
	/* detecting rising edge (finish) of negative pulse */
	else if (input > (EOG_BLINK_MIN_THRESHOLD + SHMITH_SHIFT)
			&& exg.EOGNegativePulseDetectionLock == 1) {
		exg.EOGNegativePulseDetectionLock = 0;
		exg.EOGNegativePulseDetectionTick = HAL_GetTick();
		exg.EOGNegativePulseDetectionFlag = 1;
		if (exg.EOGPositivePulseDetectionFlag == 1) {
			if (exg.EOGNegativePulseDetectionTick
					- exg.EOGPositivePulseDetectionTick< EOG_ONE_BLINK_PERIOD_MS) {
				exg.eyeBlinkStatus = RIGHT_BLINK;
				exg.EOGNegativePulseDetectionFlag = 0;
				exg.EOGPositivePulseDetectionFlag = 0;
			} else
				exg.EOGPositivePulseDetectionFlag = 0;
		}
	}
}

/***************************************************************************/
/*  */
Module_Status EXG_SignalProcessing(void) {
	uint8_t status = H2BR0_OK;
	LeadsStatus_EXG leadsStatus;
	InputSignal_EXG inputSignal;

	CheckLeadsStatus(&leadsStatus);

	exg.analogSample = (float) (exg.AdcValue) / ADC_NUM_OF_STATES * ADC_VREF; // Convert to analog: 12bit, Vref=3.3V
	inputSignal = exg.inputSignalType;

	switch (inputSignal) {
	case ECG:
		ECG_Filter();
		ECG_BaselineFilter();
		if (exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
			ECG_HeartRateCalculation();
		break;

	case EOG:
		EOG_Filter();
		if (exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
			EyeBlinkDetection();
		break;

	case EEG:
		EEG_Filter();
		break;

	case EMG:
		EMG_Filter();
		if (exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES) {
			EMG_Rectifying();
			EMG_EnvelopeDetection();
			EMG_PulseDetection();
		}
		break;

	default:
		status = H2BR0_ERR_WRONGPARAMS;
	}
	if (plotEnabled == true) {
		PlotToTerminal(plotPort);
	}
	return status;
}

/***************************************************************************/

/* timer2 EXG special timer callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == EXG_TIM) {
		end_time = HAL_GetTick();
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		Loop_time = end_time - start_time;
		SetSamplingFlag();
		vTaskNotifyGiveFromISR(EXGSignalProcessingHandle,
				&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		start_time = HAL_GetTick();
		EXG_SignalProcessing();
	}
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/*
 * @brief: Initialize the signal type to be measured.
 * @param1: inputSignal to specify signal type (EMG - ECG - EEG - EOG).
 * @retval: status
 */
Module_Status EXG_Init(InputSignal_EXG inputSignal) {
	uint8_t status = H2BR0_OK;

	EXG_Enable();

	switch (inputSignal) {
	case ECG:
		EXG_TIM_PERIOD = ECG_SAMPLE_TIME;
		exg.inputSignalType = inputSignal;
		break;

	case EOG:
		EXG_TIM_PERIOD = EOG_SAMPLE_TIME;
		exg.inputSignalType = inputSignal;
		break;

	case EEG:
		EXG_TIM_PERIOD = EEG_SAMPLE_TIME;
		exg.inputSignalType = inputSignal;
		break;

	case EMG:
		EXG_TIM_PERIOD = EMG_SAMPLE_TIME;
		exg.inputSignalType = inputSignal;
		exg.EMGPulseDetectionThreshold = EMG_PULSE_MAX_THRESHOLD;
		/* so that no pulse detected until adjusting threshold by the user */
		break;

	default:
		status = H2BR0_ERR_WRONGPARAMS;
		break;
	}
	Delay_ms(2000); /* avoiding transient state when module is power on */

	HAL_TIM_Base_Start_IT(&HANDLER_Timer_EXG);
	HAL_ADC_Start_DMA(&HANDLER_ADC_EXG, &(exg.AdcValue), 1);

	return status;
}
/***************************************************************************/
/* Enabling signal plot on a given port.
 * port: The communication port (e.g., UART, USB) to send the plotted data.
 */
Module_Status EnablePlot(uint8_t port) {
	Module_Status status = H2BR0_OK;

	plotPort = port;     // Store the selected port
	plotEnabled = true;  // Enable plotting flag
	HAL_UART_DMAPause(GetUart(port));

	return status;
}

/***************************************************************************/
/* Disabling signal plot.
 * This stops sending data for plotting.
 */
Module_Status DisablePlot(uint8_t port) {
	Module_Status status = H2BR0_OK;

	plotEnabled = false;  // Disable plotting flag
	HAL_UART_DMAResume(GetUart(port));
	return status;
}

/***************************************************************************/
/* Setting the threshold for EMG signal.
 * threshold: value (0 - 100).
 */
Module_Status EMG_SetThreshold(uint8_t threshold) {
	uint8_t status = H2BR0_OK;
	float voltThreshold;

	if (exg.inputSignalType == EMG) {
		if (threshold > 100)
			threshold = 100;	// threshold = [0,100]
		voltThreshold = ((EMG_PULSE_MAX_THRESHOLD - EMG_PULSE_MIN_THRESHOLD)
				/ 100.0) * (float) threshold + EMG_PULSE_MIN_THRESHOLD; // mapping from [0,100] to [EMG_PULSE_MIN_THRESHOLD, EMG_PULSE_MAX_THRESHOLD]
		exg.EMGPulseDetectionThreshold = voltThreshold;
	} else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/***************************************************************************/
/* reading the time of how long the EMG signal lasted with the threshold value.
 * EMGDetectionFlag: pointer to a buffer to store value.
 * EMGDurationMsec: pointer to a buffer to store value.
 */
Module_Status EMG_CheckPulse(uint8_t *EMGDetectionFlag,
		uint16_t *EMGDurationMsec) {
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EMG) {
		*EMGDetectionFlag = exg.EMGPulseDetectionFlag;
		if (*EMGDetectionFlag == 1) {
			*EMGDurationMsec = exg.EMGPulseDurationMsec;
			exg.EMGPulseDetectionFlag = 0;
		}
	} else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/***************************************************************************/
/* reading eye movement state (rapid right or left - up or down) based on electrode placement.
 * eyeBlinkStatus: pointer to a buffer to store value.
 */
Module_Status EOG_CheckEyeBlink(EyeBlinkingStatus *eyeBlinkStatus) {
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EOG) {
		*eyeBlinkStatus = exg.eyeBlinkStatus;
		if (*eyeBlinkStatus != NO_BLINK)
			exg.eyeBlinkStatus = NO_BLINK;
	} else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/***************************************************************************/
/* Extracting a normal sample and a filtered sample from the ECG signal.
 * sample pointer to a buffer to store value.
 * filteredSample pointer to a buffer to store value.
 */

Module_Status ECG_Sample(float *sample, float *filteredSample) {
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	GetSamplingFlag(&samplingFlag);

	if (exg.inputSignalType == ECG && samplingFlag == 1) {
		sample[ECG_Index] = exg.analogSample;
		filteredSample[ECG_Index] = exg.filteredSample;

		ECG_Index++;
		if (ECG_Index >= ECG_BUF_LEN) {
			ECG_Index = 0;
		}

		ResetSamplingFlag();
	} else {
		status = H2BR0_ERR_WRONGPARAMS;
	}

	return status;
}

/***************************************************************************/
/* Extracting a normal sample and a filtered sample from the EOG signal.
 * sample: pointer to a buffer to store value.
 * filteredSample: pointer to a buffer to store value.
 */

Module_Status EOG_Sample(float *sample, float *filteredSample) {
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	GetSamplingFlag(&samplingFlag);

	if (exg.inputSignalType == EOG && samplingFlag == 1) {
		sample[EOG_Index] = exg.analogSample;
		filteredSample[EOG_Index] = exg.filteredSample;

		EOG_Index++;
		if (EOG_Index >= EOG_BUF_LEN) {
			EOG_Index = 0;
		}

		ResetSamplingFlag();
	} else {
		status = H2BR0_ERR_WRONGPARAMS;
	}

	return status;
}

/***************************************************************************/
/* Extracting a normal sample and a filtered sample from the EEG signal.
 * sample: pointer to a buffer to store value.
 * filteredSample: pointer to a buffer to store value.
 */

Module_Status EEG_Sample(float *sample, float *filteredSample) {
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	GetSamplingFlag(&samplingFlag);

	if (exg.inputSignalType == EEG && samplingFlag == 1) {
		sample[EEG_Index] = exg.analogSample;
		filteredSample[EEG_Index] = exg.filteredSample;

		EEG_Index++;
		if (EEG_Index >= EEG_BUF_LEN) {
			EEG_Index = 0;
		}

		ResetSamplingFlag();
	} else {
		status = H2BR0_ERR_WRONGPARAMS;
	}

	return status;
}

/***************************************************************************/
/* Extracting a normal sample, a filtered sample, a rectified sample,
 * and an envelope sample from the EMG signal.
 * sample: pointer to a buffer to store value.
 * filteredSample: pointer to a buffer to store value.
 * rectifiedSample: pointer to a buffer to store value.
 * envelopeSample: pointer to a buffer to store value.
 */

Module_Status EMG_Sample(float *sample, float *filteredSample,
		float *rectifiedSample, float *envelopeSample) {
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	GetSamplingFlag(&samplingFlag);

	if (exg.inputSignalType == EMG && samplingFlag == 1) {
		sample[EMG_Index] = exg.analogSample;
		filteredSample[EMG_Index] = exg.filteredSample;
		rectifiedSample[EMG_Index] = exg.EMGRectifiedSample;
		envelopeSample[EMG_Index] = exg.EMGEnvelopeSample;

		EMG_Index++;
		if (EMG_Index >= EMG_BUF_LEN) {
			EMG_Index = 0;
		}

		ResetSamplingFlag();
	} else {
		status = H2BR0_ERR_WRONGPARAMS;
	}

	return status;
}

/***************************************************************************/
/* reading heart rate from the ECG signal.
 * heartRate: pointer to a buffer to store value
 */
Module_Status ECG_HeartRate(uint8_t *heartRate) {
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == ECG)
		*heartRate = exg.heartRate;
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/***************************************************************************/
/* Sending (normal sample) and (filtered sample) to display on Terminal or draw
 * signals for EMG,EEG,ECG,EOG
 * port: The port you want to send from
 */
Module_Status PlotToTerminal(uint8_t port) {
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	char sendData[80] = { 0 };
	if (port == 0)
		return H2BR0_ERR_WRONGPARAMS;

	if (exg.inputSignalType == EMG) {

		sprintf(sendData, "%5.2f,%5.2f,%5.2f,%5.2f\r\n", exg.analogSample,
				exg.filteredSample, exg.EMGRectifiedSample,
				exg.EMGEnvelopeSample);

	} else {
		memset(sendData, 0, sizeof(sendData));
		sprintf(sendData, "%5.2f , %5.2f\r\n", exg.analogSample,
				exg.filteredSample);
	}

	GetSamplingFlag(&samplingFlag);

	if (samplingFlag == 1) {
		ResetSamplingFlag();
		writePxMutex(port, sendData, strlen(sendData), cmd50ms, 10);
	}

	return status;
}

/***************************************************************************/
/* reading Electrodes status.
 * leadsStatus: pointer to a buffer to store value
 */
Module_Status LeadsStatus(LeadsStatus_EXG *leadsStatus) {
	uint8_t status = H2BR0_OK;

	*leadsStatus = exg.statusOfLeads;
	return status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
