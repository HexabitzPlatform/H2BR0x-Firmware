/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H2BR0.c
 Description   : Source code for module H2BR0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H2BR0_inputs.h"

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};
EXG_t exg;
/* exported functions */

/* Private variables ---------------------------------------------------------*/
TaskHandle_t EXGTaskHandle = NULL;

uint8_t emgFlag;
uint16_t emgDurMsec;
uint8_t heartrate;
uint32_t x=0;
uint16_t i;
uint32_t lasttick;
uint16_t rightBlinkCounte, leftBlinkCounte;

float ecgSample;
float ecgFilteredSample;
float eogSample;
float eogFilteredSample;
float emgSample;
float emgFilteredSample;
float emgFilteredSample;
float emgRectifiedSample;
float emgEnvelopeSample;



/* Private function prototypes -----------------------------------------------*/
void ExecuteMonitor(void);
void EXGTask(void *argument);

void EXG_Enable(EXG_t *EXGStruct);
void EXG_Disable(EXG_t *EXGStruct);
void EXG_Reset(EXG_t *EXGStruct);
void GetSamplingFlag(EXG_t *EXGStruct, uint8_t *samplingFlag);
void ResetSamplingFlag(EXG_t *EXGStruct);
void SetSamplingFlag(EXG_t *EXGStruct);
void ECG_Filter(EXG_t *EXGStruct);
void ECG_BaselineFilter(EXG_t *EXGStruct);
void ECG_HeartRateCalculation(EXG_t *EXGStruct);
void EOG_Filter(EXG_t *EXGStruct);
void EEG_Filter(EXG_t *EXGStruct);
void EMG_Filter(EXG_t *EXGStruct);
void EyeBlinkDetection(EXG_t *EXGStruct);
void EMG_Rectifying(EXG_t *EXGStruct);
void EMG_EnvelopeDetection(EXG_t *EXGStruct);
void CheckLeadsStatus(EXG_t *EXGStruct, LeadsStatus_EXG *leadsStatus);


/* Create CLI commands --------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |						    	 Private Functions						 |
 -------------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,              //HALFWORD
						//TOBECHECKED
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
		}
	}
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
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

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H2BR0 module initialization.
 */
void Module_Peripheral_Init(void){

	 __HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	MX_TIM2_Init();

	 //Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		}
	}

	/* Create module special task (if needed) */
	if(EXGTaskHandle == NULL)
		xTaskCreate(EXGTask,(const char* ) "EXGTask",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&EXGTaskHandle);

}

/*-----------------------------------------------------------*/
/* --- H2BR0 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H2BR0_OK;


	switch(code){

		default:
			result =H2BR0_ERR_UnknownMessage;
			break;
	}
	
	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART6)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	else if(huart->Instance == USART6)
		return P6;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){

}

/*-----------------------------------------------------------*/

/* Module special task function (if needed) */
void EXGTask(void *argument){

	EyeBlinkingStatus eyeBlinkStatus;
	LeadsStatus_EXG wiresStatus;

	/* Infinite loop */
	uint8_t cases; // Test variable.


	for(;;){
		/*  */

//#ifdef ECG_Signal
/**************** Application for ECG *******************/
	LeadsStatus(&wiresStatus);
	PlotToTerminal(&huart3);
	ECG_HeartRate(&heartrate);
/****************************************************/
//#endif


//#ifdef EOG_Signal
/**************** Application for EOG *******************/

	LeadsStatus(&wiresStatus);
	PlotToTerminal(&huart3);
	CheckEyeBlink(&eyeBlinkStatus);
	if(eyeBlinkStatus == RIGHT_BLINK)
		rightBlinkCounte ++;
	else if (eyeBlinkStatus == LEFT_BLINK)
	    leftBlinkCounte ++;
/****************************************************/
//#endif


//#ifdef EMG_Signal
/**************** Application for EMG *******************/
	  EMG_CheckPulse(&emgFlag, &emgDurMsec);
	  if (emgFlag == 1)
	  {
		  if(emgDurMsec>150)
		  {
//			  HAL_GPIO_WritePin(GPIOB,MOTOR_CONTROL_Pin, GPIO_PIN_SET);
			  x=1;
			  HAL_Delay(5000);
//			  HAL_GPIO_WritePin(GPIOB,MOTOR_CONTROL_Pin, GPIO_PIN_RESET);
			  x=0;
		  }
	  }
	  LeadsStatus(&wiresStatus);

/****************************************************/
//#endif


		switch(cases){


			default:
				osDelay(10);
				break;
		}

		taskYIELD();
	}

}


/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |							 	Local  APIs			    		          | 																 	|
/* -----------------------------------------------------------------------
 */
void EXG_Enable(EXG_t *EXGStruct)
{
	HAL_GPIO_WritePin(SDN_EXG_GPIO_Port, SDN_EXG_Pin, GPIO_PIN_SET);
	EXGStruct->EXGStatus = EXG_ENABLED;
}
/*-----------------------------------------------------------*/
void EXG_Disable(EXG_t *EXGStruct)
{
	HAL_GPIO_WritePin(SDN_EXG_GPIO_Port, SDN_EXG_Pin, GPIO_PIN_RESET);
	EXGStruct->EXGStatus = EXG_DISABLED;
}
/*-----------------------------------------------------------*/
void EXG_Reset(EXG_t *EXGStruct)
{
	EXG_Disable(EXGStruct);
	HAL_Delay(10);
	EXG_Enable(EXGStruct);
}
/*-----------------------------------------------------------*/
void GetSamplingFlag(EXG_t *EXGStruct, uint8_t *samplingFlag)
{
	*samplingFlag = EXGStruct->samplingFlag;
}
/*-----------------------------------------------------------*/
void ResetSamplingFlag(EXG_t *EXGStruct)
{
	EXGStruct->samplingFlag = 0;
}
/*-----------------------------------------------------------*/
void SetSamplingFlag(EXG_t *EXGStruct)
{
	EXGStruct->samplingFlag = 1;
}
/*-----------------------------------------------------------*/
void CheckLeadsStatus(EXG_t *EXGStruct, LeadsStatus_EXG *leadsStatus)
{
	GPIO_PinState LODPStatus;
	GPIO_PinState LODNStatus;
	LODPStatus = HAL_GPIO_ReadPin(LODP_EXG_GPIO_Port, LODP_EXG_Pin);
	LODNStatus = HAL_GPIO_ReadPin(LODN_EXG_GPIO_Port, LODN_EXG_Pin);
	if (LODPStatus == GPIO_PIN_RESET && LODNStatus == GPIO_PIN_RESET)
		*leadsStatus = LeadP_CONNECTED_LeadN_CONNECTED;
	else if (LODPStatus == GPIO_PIN_RESET && LODNStatus == GPIO_PIN_SET)
		*leadsStatus = LeadP_CONNECTED_LeadN_NOTCONNECTED;
	else if (LODPStatus == GPIO_PIN_SET && LODNStatus == GPIO_PIN_RESET)
		*leadsStatus = LeadP_NOTCONNECTED_LeadN_CONNECTED;
	else *leadsStatus = LeadP_NOTCONNECTED_LeadN_NOTCONNECTED;
	EXGStruct->statusOfLeads = *leadsStatus;
}
/*-----------------------------------------------------------*/
void ECG_Filter(EXG_t *EXGStruct)
{
	float input1 = EXGStruct->analogSample;
	float preInput1 = EXGStruct->tempFilterInputBuffer[0];
	float beforePreInput1 = EXGStruct->tempFilterInputBuffer[1];
	float output1;
	float preOutput1 = EXGStruct->tempFilterOutputBuffer[0];
	float beforePreOutput1 = EXGStruct->tempFilterOutputBuffer[1];
	// LPF: Fs=120sps, Fc=40Hz, Order=2
	output1= -0.6202 * preOutput1 - 0.2404 * beforePreOutput1 + 0.4652 * input1 + 0.9303 * preInput1 + 0.4652 * beforePreInput1;
	beforePreInput1 = preInput1;
	preInput1 = input1;
	beforePreOutput1 = preOutput1;
	preOutput1= output1;
	EXGStruct->tempFilterInputBuffer[0] = preInput1;
	EXGStruct->tempFilterInputBuffer[1] = beforePreInput1;
	EXGStruct->tempFilterOutputBuffer[0] = preOutput1;
	EXGStruct->tempFilterOutputBuffer[1] = beforePreOutput1;

	float input2 = output1;
	float preInput2 = EXGStruct->tempFilterInputBuffer[2];
	float beforePreInput2 = EXGStruct->tempFilterInputBuffer[3];
	float output2;
	float preOutput2 = EXGStruct->tempFilterOutputBuffer[2];
	float beforePreOutput2 = EXGStruct->tempFilterOutputBuffer[3];
	// LPF: Fs=120sps, Fc=40Hz, Order=2
	output2= -0.6202 * preOutput2 - 0.2404 * beforePreOutput2 + 0.4652 * input2 + 0.9303 * preInput2 + 0.4652 * beforePreInput2;
	beforePreInput2 = preInput2;
	preInput2 = input2;
	beforePreOutput2 = preOutput2;
	preOutput2= output2;
	EXGStruct->tempFilterInputBuffer[2] = preInput2;
	EXGStruct->tempFilterInputBuffer[3] = beforePreInput2;
	EXGStruct->tempFilterOutputBuffer[2] = preOutput2;
	EXGStruct->tempFilterOutputBuffer[3] = beforePreOutput2;
	EXGStruct->filteredSample = output2;
}
/*-----------------------------------------------------------*/
void EOG_Filter(EXG_t *EXGStruct)
{
	float input = EXGStruct->analogSample;
	float preInput = EXGStruct->tempFilterInputBuffer[0];
	float beforePreInput = EXGStruct->tempFilterInputBuffer[1];
	float output;
	float preOutput = EXGStruct->tempFilterOutputBuffer[0];
	float beforePreOutput = EXGStruct->tempFilterOutputBuffer[1];
	// LPF: Fs=100sps, Fc=25Hz, Order=2
	output= -0.0 * preOutput - 0.1716 * beforePreOutput + 0.2929 * input + 0.5858 * preInput + 0.2929 * beforePreInput;
	beforePreInput = preInput;
	preInput = input;
	beforePreOutput = preOutput;
	preOutput= output;
	EXGStruct->tempFilterInputBuffer[0] = preInput;
	EXGStruct->tempFilterInputBuffer[1] = beforePreInput;
	EXGStruct->tempFilterOutputBuffer[0] = preOutput;
	EXGStruct->tempFilterOutputBuffer[1] = beforePreOutput;
	EXGStruct->filteredSample = output;
}
/*-----------------------------------------------------------*/
void EEG_Filter(EXG_t *EXGStruct)
{
	float input = EXGStruct->analogSample;
	float preInput = EXGStruct->tempFilterInputBuffer[0];
	float beforePreInput = EXGStruct->tempFilterInputBuffer[1];
	float output;
	float preOutput = EXGStruct->tempFilterOutputBuffer[0];
	float beforePreOutput = EXGStruct->tempFilterOutputBuffer[1];
	// LPF: Fs=100sps, Fc=30Hz, Order=2
	output= -0.3695 * preOutput - 0.1958 * beforePreOutput + 0.3913 * input + 0.7827 * preInput + 0.3913 * beforePreInput;
	beforePreInput = preInput;
	preInput = input;
	beforePreOutput = preOutput;
	preOutput= output;
	EXGStruct->tempFilterInputBuffer[0] = preInput;
	EXGStruct->tempFilterInputBuffer[1] = beforePreInput;
	EXGStruct->tempFilterOutputBuffer[0] = preOutput;
	EXGStruct->tempFilterOutputBuffer[1] = beforePreOutput;
	EXGStruct->filteredSample = output;
}
/*-----------------------------------------------------------*/
void EMG_Filter(EXG_t *EXGStruct)
{
	float input1 = EXGStruct->analogSample;
	float preInput1 = EXGStruct->tempFilterInputBuffer[0];
	float beforePreInput1 = EXGStruct->tempFilterInputBuffer[1];
	float output1;
	float preOutput1 = EXGStruct->tempFilterOutputBuffer[0];
	float beforePreOutput1 = EXGStruct->tempFilterOutputBuffer[1];
	// LPF: Fs=500sps, Fc=150Hz, Order=2
	output1= -0.3695 * preOutput1 - 0.1958 * beforePreOutput1 + 0.3913 * input1 + 0.7827 * preInput1 + 0.3913 * beforePreInput1;
	beforePreInput1 = preInput1;
	preInput1 = input1;
	beforePreOutput1 = preOutput1;
	preOutput1= output1;
	EXGStruct->tempFilterInputBuffer[0] = preInput1;
	EXGStruct->tempFilterInputBuffer[1] = beforePreInput1;
	EXGStruct->tempFilterOutputBuffer[0] = preOutput1;
	EXGStruct->tempFilterOutputBuffer[1] = beforePreOutput1;

	float input2 = output1;
	float preInput2 = EXGStruct->tempFilterInputBuffer[2];
	float beforePreInput2 = EXGStruct->tempFilterInputBuffer[3];
	float output2;
	float preOutput2 = EXGStruct->tempFilterOutputBuffer[2];
	float beforePreOutput2 = EXGStruct->tempFilterOutputBuffer[3];
	// HPF: Fs=500sps, Fc=20Hz, Order=2
	output2= 1.6475 * preOutput2 - 0.7009 * beforePreOutput2 + 0.8371 * input2 - 1.6742 * preInput2 + 0.8371 * beforePreInput2;
	beforePreInput2 = preInput2;
	preInput2 = input2;
	beforePreOutput2 = preOutput2;
	preOutput2= output2;
	EXGStruct->tempFilterInputBuffer[2] = preInput2;
	EXGStruct->tempFilterInputBuffer[3] = beforePreInput2;
	EXGStruct->tempFilterOutputBuffer[2] = preOutput2;
	EXGStruct->tempFilterOutputBuffer[3] = beforePreOutput2;
	EXGStruct->filteredSample = output2;
}
/*-----------------------------------------------------------*/
void ECG_BaselineFilter(EXG_t *EXGStruct)
{
	float input = EXGStruct->filteredSample;
	float preInput = EXGStruct->tempFilterInputBuffer[4];
	float output;
	float preOutput = EXGStruct->tempFilterOutputBuffer[4];

	// HPF: Fs=120sps, Fc=7Hz, Order=1
	output= 0.6873 * preOutput + 0.8436 * input - 0.8436 * preInput;
	preInput = input;
	preOutput= output;
	EXGStruct->tempFilterInputBuffer[4] = preInput;
	EXGStruct->tempFilterOutputBuffer[4] = preOutput;
	EXGStruct->ECGBaselineFilteredSample = output;
}
/*-----------------------------------------------------------*/
void ECG_HeartRateCalculation(EXG_t *EXGStruct)
{
	float input = EXGStruct->ECGBaselineFilteredSample;
	uint16_t period;
	float HR;
	float heartRateSum = 0;
	if (input >= ECG_THRESHOLD && EXGStruct->heartRateLock == 0)
	{
		EXGStruct->heartRateLock = 1;
		period = HAL_GetTick() - EXGStruct->HRCalculationLastTick;    // find time between tow beats in msec.
		HR = 60000.0 / (float)period;
		if(HR >= HEART_RATE_MIN && HR <= HEART_RATE_MAX)
		{
			// if relative change between current HR and old HR within specific range, send HR else ignore sending current value (there is noise)
			if ((HR >= 0.8 * EXGStruct->previousHeartRate) && (HR <= 1.2 * EXGStruct->previousHeartRate))
				{
					EXGStruct->heartRateArray[EXGStruct->heartRateIndex ++] = HR;
					if(EXGStruct->heartRateIndex == HEART_RATE_ARRAY_SIZE)
					{
						EXGStruct->heartRateIndex = 0;
						for (uint8_t i=0; i<HEART_RATE_ARRAY_SIZE; i++)
							heartRateSum += EXGStruct->heartRateArray[i];
						EXGStruct->heartRate = roundf (heartRateSum / HEART_RATE_ARRAY_SIZE);
					}
				}
		}
		else
		{
			HR = 0;
			EXGStruct->heartRate = HR;
			EXGStruct->heartRateIndex = 0;  // empty heartRateArray when happening wrong heart rate
		}
		EXGStruct->previousHeartRate = HR;
		EXGStruct->HRCalculationLastTick = HAL_GetTick();
	}
	else if(input < ECG_THRESHOLD)
		EXGStruct->heartRateLock = 0;
}
/*-----------------------------------------------------------*/
void EOG_EnvelopeDetection(EXG_t *EXGStruct)
{
	float input = EXGStruct->EMGRectifiedSample;
	uint8_t index = EXGStruct->windowBufferIndex;
	float lastSampleInWindow = EXGStruct->movingWindowBuffer[index];
	float sum = EXGStruct->sumOfSamplesValuesInWindow;
	float movingMean;
	sum = sum - lastSampleInWindow;
	sum = sum + input;  // new sample get into window
	EXGStruct->movingWindowBuffer[index] = input;
	index ++;
	if (index == EMG_MOVING_WINDOW) // circular buffer
		index = 0;
	movingMean = (sum / EMG_MOVING_WINDOW) * EMG_EVELOPE_GAIN_FACTOR;
	EXGStruct->sumOfSamplesValuesInWindow = sum;
	EXGStruct->EMGEnvelopeSample = movingMean;
	EXGStruct->windowBufferIndex = index;
}
/*-----------------------------------------------------------*/
void EMG_Rectifying(EXG_t *EXGStruct)
{
	float input = EXGStruct->filteredSample;
	float absInput = input;
	if (input < 0.0)
		absInput = - input;
	EXGStruct->EMGRectifiedSample = absInput;
}
/*-----------------------------------------------------------*/
void EMG_EnvelopeDetection(EXG_t *EXGStruct)
{
	float input = EXGStruct->EMGRectifiedSample;
	uint8_t index = EXGStruct->windowBufferIndex;
	float lastSampleInWindow = EXGStruct->movingWindowBuffer[index];
	float sum = EXGStruct->sumOfSamplesValuesInWindow;
	float movingMean;
	sum = sum - lastSampleInWindow;
	sum = sum + input;  // new sample get into window
	EXGStruct->movingWindowBuffer[index] = input;
	index ++;
	if (index == EMG_MOVING_WINDOW) // circular buffer
		index = 0;
	movingMean = (sum / EMG_MOVING_WINDOW) * EMG_EVELOPE_GAIN_FACTOR;
	EXGStruct->sumOfSamplesValuesInWindow = sum;
	EXGStruct->EMGEnvelopeSample = movingMean;
	EXGStruct->windowBufferIndex = index;
}
/*-----------------------------------------------------------*/
void EMG_PulseDetection(EXG_t *EXGStruct)
{
	float input = EXGStruct->EMGEnvelopeSample;
	float EMGPulseThreshold = EXGStruct->EMGPulseDetectionThreshold;
	if (input >= EMGPulseThreshold &&  EXGStruct->EMGPulseDetectionLock == 0) // detecting rising edge of pulse
	{
		EXGStruct->EMGPulseRisingEdgeTick = HAL_GetTick();
		EXGStruct->EMGPulseDetectionLock = 1;
	}
	else if (input < EMGPulseThreshold &&  EXGStruct->EMGPulseDetectionLock == 1) // detecting falling edge of pulse
	{

		uint16_t EMGPulseTime = HAL_GetTick() - EXGStruct->EMGPulseRisingEdgeTick;
		if (EMGPulseTime > EMG_NOISY_PULSE_PERIOD_MS)
		{
			EXGStruct->EMGPulseDurationMsec = EMGPulseTime;
			EXGStruct->EMGPulseDetectionFlag = 1;
		}
		EXGStruct->EMGPulseDetectionLock = 0;
	}
}
/*-----------------------------------------------------------*/
void EyeBlinkDetection(EXG_t *EXGStruct)
{
	float input = EXGStruct->filteredSample;

	if (input >= EOG_BLINK_MAX_THRESHOLD   &&  EXGStruct->EOGPositivePulseDetectionLock == 0) // detecting rising edge (start) of positive pulse
			EXGStruct->EOGPositivePulseDetectionLock = 1;
	else if (input < (EOG_BLINK_MAX_THRESHOLD - SHMITH_SHIFT)  &&  EXGStruct->EOGPositivePulseDetectionLock == 1) // detecting falling edge (finish) of positive pulse
	{
		EXGStruct->EOGPositivePulseDetectionLock = 0;
		EXGStruct->EOGPositivePulseDetectionTick = HAL_GetTick();
		EXGStruct->EOGPositivePulseDetectionFlag = 1;
		if(EXGStruct->EOGNegativePulseDetectionFlag == 1)
		{
			if(EXGStruct->EOGPositivePulseDetectionTick - EXGStruct->EOGNegativePulseDetectionTick < EOG_ONE_BLINK_PERIOD_MS)
				{
					EXGStruct->eyeBlinkStatus = LEFT_BLINK;
					EXGStruct->EOGNegativePulseDetectionFlag = 0;
					EXGStruct->EOGPositivePulseDetectionFlag = 0;
				}
			else EXGStruct->EOGNegativePulseDetectionFlag = 0;
		}
	}
	else if (input <= EOG_BLINK_MIN_THRESHOLD  &&  EXGStruct->EOGNegativePulseDetectionLock == 0) //detecting falling edge (start) of negative pulse
		EXGStruct->EOGNegativePulseDetectionLock = 1;
	else if (input > (EOG_BLINK_MIN_THRESHOLD + SHMITH_SHIFT)  &&  EXGStruct->EOGNegativePulseDetectionLock == 1 ) //detecting rising edge (finish) of negative pulse
	{
		EXGStruct->EOGNegativePulseDetectionLock = 0;
		EXGStruct->EOGNegativePulseDetectionTick = HAL_GetTick();
		EXGStruct->EOGNegativePulseDetectionFlag = 1;
		if(EXGStruct->EOGPositivePulseDetectionFlag ==1)
		{
			if(EXGStruct->EOGNegativePulseDetectionTick - EXGStruct->EOGPositivePulseDetectionTick < EOG_ONE_BLINK_PERIOD_MS)
				{
					EXGStruct->eyeBlinkStatus = RIGHT_BLINK;
					EXGStruct->EOGNegativePulseDetectionFlag = 0;
					EXGStruct->EOGPositivePulseDetectionFlag = 0;
				}
			else EXGStruct->EOGPositivePulseDetectionFlag = 0;
		}
	}
}

/*-----------------------------------------------------------*/
/* timer2 EXG special timer callback */
void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef* htim)
{

	if(htim->Instance == EXG_TIM)
	{
		SetSamplingFlag(&exg);
		EXG_SignalProcessing();
	}
	x++;
}

/* -----------------------------------------------------------------------
 |								  APIs							          |
/* -----------------------------------------------------------------------
 */
/*       */
Module_Status EXG_Init(InputSignal_EXG inputSignal)
{
	uint8_t status = H2BR0_OK;

	EXG_Enable(&exg);

	switch (inputSignal)
	{
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
			status = H2BR0_ERR_WrongParams;
			break;
	}
	Delay_ms(2000);  // avoiding transient state when module is power on
	HAL_TIM_Base_Start_IT(&HANDLER_Timer_EXG);
	HAL_ADC_Start_DMA(&HANDLER_ADC_EXG, &(exg.AdcValue), 1);

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status EXG_SignalProcessing(void)
{
	uint8_t status = H2BR0_OK;
	LeadsStatus_EXG leadsStatus;
	InputSignal_EXG inputSignal;

	CheckLeadsStatus(&exg, &leadsStatus);

	if (leadsStatus == LeadP_CONNECTED_LeadN_CONNECTED)
	{
		exg.analogSample = (float)(exg.AdcValue) / ADC_NUM_OF_STATES * ADC_VREF; // Convert to analog: 12bit, Vref=3.3V
		inputSignal = exg.inputSignalType;

		switch (inputSignal)
		{
			case ECG:
				ECG_Filter(&exg);
				ECG_BaselineFilter(&exg);
				if(exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
					ECG_HeartRateCalculation(&exg);
				break;

			case EOG:
				EOG_Filter(&exg);
				if(exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
					EyeBlinkDetection(&exg);
				break;

			case EEG:
				EEG_Filter(&exg);
				break;

			case EMG:
				EMG_Filter(&exg);
				if(exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
				{
					EMG_Rectifying(&exg);
					EMG_EnvelopeDetection(&exg);
					EMG_PulseDetection(&exg);
				}
				break;

			default:
				status = H2BR0_ERR_WrongParams;
		}
	}
	else
		status = H2BR0_ERR_LEADS_NOTCONNECTED;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status EMG_SetThreshold(uint8_t threshold)
{
	uint8_t status = H2BR0_OK;
	float voltThreshold;

	if (exg.inputSignalType == EMG)
	{
		if (threshold > 100)
			threshold = 100;	// threshold = [0,100]
		voltThreshold = ((EMG_PULSE_MAX_THRESHOLD - EMG_PULSE_MIN_THRESHOLD)/ 100.0) * (float)threshold  + EMG_PULSE_MIN_THRESHOLD; // mapping from [0,100] to [EMG_PULSE_MIN_THRESHOLD, EMG_PULSE_MAX_THRESHOLD]
		exg.EMGPulseDetectionThreshold = voltThreshold;
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status EMG_CheckPulse(uint8_t *EMGDetectionFlag, uint16_t *EMGDurationMsec)
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EMG)
	{
		*EMGDetectionFlag = exg.EMGPulseDetectionFlag;
		if (*EMGDetectionFlag == 1)
		{
			*EMGDurationMsec = exg.EMGPulseDurationMsec;
			exg.EMGPulseDetectionFlag = 0;
		}
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status CheckEyeBlink(EyeBlinkingStatus *eyeBlinkStatus)
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EOG)
	{
		*eyeBlinkStatus = exg.eyeBlinkStatus;
		if (*eyeBlinkStatus != NO_BLINK)
			exg.eyeBlinkStatus = NO_BLINK;
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status ECG_Sample(float *sample, float *filteredSample )
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == ECG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status EOG_Sample(float *sample, float *filteredSample )
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EOG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status EEG_Sample(float *sample, float *filteredSample )
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EEG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status EMG_Sample(float *sample, float *filteredSample, float *rectifiedSample, float *envelopeSample)
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EMG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
		*rectifiedSample = exg.EMGRectifiedSample;
		*envelopeSample =  exg.EMGEnvelopeSample;
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status ECG_HeartRate(uint8_t *heartRate)
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == ECG)
		*heartRate = exg.heartRate;
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status PlotToTerminal(UART_HandleTypeDef *huart)
{
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	char sendData[26];

	if(exg.inputSignalType == ECG || exg.inputSignalType == EOG || exg.inputSignalType == EEG || exg.inputSignalType == EMG)
	{
		if(exg.inputSignalType == EMG)
			sprintf(sendData, "a%5.2fb%5.2fc%5.2fd%5.2f\n",exg.analogSample, exg.filteredSample, exg.EMGRectifiedSample, exg.EMGEnvelopeSample);
		else
			sprintf(sendData, "a%5.2fb%5.2f\n", exg.analogSample, exg.filteredSample);

		GetSamplingFlag(&exg, &samplingFlag);

		if (samplingFlag == 1)
		{
			ResetSamplingFlag(&exg);
		    HAL_UART_Transmit(huart, sendData, strlen(sendData), 100);
		}
	}
	else
		status = H2BR0_ERR_WrongParams;

	return status;
}

/*-----------------------------------------------------------*/
/*  */
Module_Status LeadsStatus(LeadsStatus_EXG *leadsStatus)
{
	uint8_t status = H2BR0_OK;

	*leadsStatus = exg.statusOfLeads;
	return status;
}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */



/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
