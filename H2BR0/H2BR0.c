/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
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
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};
#define MIN_PERIOD_MS				100
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule);
typedef void (*SampleMemsToString)(char *, size_t);
/* exported functions */

/* Private variables ---------------------------------------------------------*/
static bool stopStream = false;
TaskHandle_t EXGTaskHandle = NULL;
EXG_t exg;
uint8_t port1, module1,mode1;
uint8_t port2 ,module2,mode2;
uint32_t Numofsamples1 ,timeout1;
uint32_t Numofsamples2 ,timeout2;
uint8_t flag ;
uint8_t tofMode ;
/* Private function prototypes -----------------------------------------------*/
Module_Status ExportStreanToPort (uint8_t module,uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout);
Module_Status ExportStreanToTerminal (uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout);
void ExecuteMonitor(void);
void EXGTask(void *argument);
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

/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_ECG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_EOG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_EEG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_EMG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_EMG_SetThresholdCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_EMG_CheckPulseCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ECG_HeartRateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_EOG_CheckEyeBlinkCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_LeadsStatusCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE StreamEXGCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );


/*-----------------------------------------------------------*/
/* CLI command structure : ECG_Sample */
const CLI_Command_Definition_t CLI_ECG_SampleCommandDefinition =
{
	( const int8_t * ) "ecg_sample", /* The command string to type. */
	( const int8_t * ) "ecg_sample:\r\nExtracting a normal sample and a filtered sample from the ECG signal.\r\n\r\n",
	CLI_ECG_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [EMG]/[EEG]/[EOG]/[ECG] (Numofsamples ) (time in ms) [port] [module].\r\n\r\n",
	StreamEXGCommand,
	-1
};
/*-----------------------------------------------------------*/
/* CLI command structure : EOG_Sample */
const CLI_Command_Definition_t CLI_EOG_SampleCommandDefinition =
{
	( const int8_t * ) "eog_sample", /* The command string to type. */
	( const int8_t * ) "eog_sample:\r\n Extracting a normal sample and a filtered sample from the EOG signal. \r\n\r\n",
	CLI_EOG_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : EEG_Sample */
const CLI_Command_Definition_t CLI_EEG_SampleCommandDefinition =
{
	( const int8_t * ) "eeg_sample", /* The command string to type. */
	( const int8_t * ) "eeg_sample:\r\n Extracting a normal sample and a filtered sample from the EEG signal. \r\n\r\n",
	CLI_EEG_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : EMG_Sample */
const CLI_Command_Definition_t CLI_EMG_SampleCommandDefinition =
{
	( const int8_t * ) "emg_sample", /* The command string to type. */
	( const int8_t * ) "emg_sample:\r\n Extracting a normal sample, a filtered sample, a rectified sample, and an envelope sample from the EMG signal.\r\n\r\n",
	CLI_EMG_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : EMG_SetThreshold */
const CLI_Command_Definition_t CLI_EMG_SetThresholdCommandDefinition =
{
	( const int8_t * ) "emg_setthreshold", /* The command string to type. */
	( const int8_t * ) "emg_setthreshold:\r\n Seting the threshold for EMG signal.\r\n\r\n",
	CLI_EMG_SetThresholdCommand, /* The function to run. */
	1 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : EMG_CheckPulse */
const CLI_Command_Definition_t CLI_EMG_CheckPulseCommandDefinition =
{
	( const int8_t * ) "emg_checkpulse", /* The command string to type. */
	( const int8_t * ) "emg_checkpulse:\r\n reading the time of how long the EMG signal lasted with the threshold value. \r\n\r\n",
	CLI_EMG_CheckPulseCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ECG_HeartRate */
const CLI_Command_Definition_t CLI_ECG_HeartRateCommandDefinition =
{
	( const int8_t * ) "ecg_heartrate", /* The command string to type. */
	( const int8_t * ) "ecg_heartrate:\r\n reading heart rate from the ECG signal. \r\n\r\n",
	CLI_ECG_HeartRateCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : CheckEyeBlink */
const CLI_Command_Definition_t CLI_EOG_CheckEyeBlinkCommandDefinition =
{
	( const int8_t * ) "eog_checkeyeblink", /* The command string to type. */
	( const int8_t * ) "eog_checkeyeblink:\r\n reading eye movement state (rapid right or left - up or down) based on electrode placement. \r\n\r\n",
	CLI_EOG_CheckEyeBlinkCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : LeadsStatus */
const CLI_Command_Definition_t CLI_LeadsStatusCommandDefinition =
{
	( const int8_t * ) "leadsstatus", /* The command string to type. */
	( const int8_t * ) "leadsstatus:\r\n reading Electrodes status. \r\n\r\n",
	CLI_LeadsStatusCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

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
	  HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


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
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();

	 //Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		}
	}
	 xTaskCreate(EXGTask,(const char* ) "EXGTask",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&EXGTaskHandle);

}

/*-----------------------------------------------------------*/
/* --- H2BR0 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H2BR0_OK;
	uint8_t uint8Data=0;
    uint16_t uint16Data=0;
    uint8_t EMGDetectionFlag=0;
    uint16_t EMGDurationMsec=0;
    uint8_t heartRate=0;
    uint8_t module;
	switch(code){
	case CODE_H2BR0_ECG_Sample:
		{
			EXG_Init(ECG);
			SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],ECG);
			break;
		}
	case CODE_H2BR0_EOG_Sample:
		{
			EXG_Init(EOG);
			SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],EOG);
			break;
		}
	case CODE_H2BR0_EEG_Sample:
		{
			EXG_Init(EEG);
			SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],EEG);
			break;
		}
	case CODE_H2BR0_EMG_Sample:
		{
			EXG_Init(EMG);
			SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],EMG);
			break;
		}
	case CODE_H2BR0_EMG_SetThreshold:
		{
			EXG_Init(EMG);
			EMG_SetThreshold(cMessage[port-1][shift]);
			break;
		}
	case CODE_H2BR0_EMG_CheckPulse:
		{
			Module_Status status =H2BR0_OK;
			module = cMessage[port-1][shift];
			port = cMessage[port-1][1+shift];
			EXG_Init(EMG);
			status = EMG_CheckPulse(&EMGDetectionFlag,&EMGDurationMsec);
			if (H2BR0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] =FMT_UINT16;
			messageParams[2] =2;
			messageParams[3] =(uint8_t)((*(uint16_t *) &EMGDetectionFlag) >> 0);
			messageParams[4] =(uint8_t)((*(uint16_t *) &EMGDetectionFlag) >> 8);
			messageParams[5] =(uint8_t)((*(uint16_t *) &EMGDurationMsec) >> 0);
			messageParams[6] =(uint8_t)((*(uint16_t *) &EMGDurationMsec) >> 8);
			SendMessageToModule(module,CODE_PORT_FORWARD,7);
			break;
		}
	case CODE_H2BR0_ECG_HeartRate:
		{
			Module_Status status =H2BR0_OK;
			module = cMessage[port-1][shift];
		    port = cMessage[port-1][1+shift];
		    EXG_Init(ECG);
		    status = ECG_HeartRate(&heartRate);
			if (H2BR0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] =FMT_UINT8;
			messageParams[2] =1;
			messageParams[3] =(uint8_t)EMGDetectionFlag;
			SendMessageToModule(module,CODE_PORT_FORWARD,4);
			break;
		}
	case CODE_H2BR0_EOG_CheckEyeBlink:
		{

			break;
		}
	case CODE_H2BR0_LeadsStatus:
		{

			break;
		}
		default:
			result =H2BR0_ERR_UNKNOWNMESSAGE;
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
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&CLI_ECG_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_EOG_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_EEG_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_EMG_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_EMG_SetThresholdCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_EMG_CheckPulseCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ECG_HeartRateCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_EOG_CheckEyeBlinkCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_LeadsStatusCommandDefinition);
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

		switch(tofMode){
		case STREAM_TO_PORT :
			ExportStreanToPort(module2, port2, mode2, Numofsamples2, timeout2);
			break;
		case STREAM_TO_Terminal :
			ExportStreanToTerminal( port1, mode1, Numofsamples1, timeout1);
			break;

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
void EXG_Enable()
{
	HAL_GPIO_WritePin(SDN_EXG_GPIO_Port, SDN_EXG_Pin, GPIO_PIN_SET);
	exg.EXGStatus = EXG_ENABLED;
}
/*-----------------------------------------------------------*/
void EXG_Disable()
{
	HAL_GPIO_WritePin(SDN_EXG_GPIO_Port, SDN_EXG_Pin, GPIO_PIN_RESET);
	exg.EXGStatus = EXG_DISABLED;
}
/*-----------------------------------------------------------*/
void EXG_Reset()
{
	EXG_Disable();
	HAL_Delay(10);
	EXG_Enable();
}
/*-----------------------------------------------------------*/
void GetSamplingFlag(uint8_t *samplingFlag)
{
	*samplingFlag =exg.samplingFlag;
}
/*-----------------------------------------------------------*/
void ResetSamplingFlag()
{
	exg.samplingFlag = 0;
}
/*-----------------------------------------------------------*/
void SetSamplingFlag()
{
	exg.samplingFlag = 1;
}
/*-----------------------------------------------------------*/
void CheckLeadsStatus(LeadsStatus_EXG *leadsStatus)
{
	GPIO_PinState LODPStatus;
	GPIO_PinState LODNStatus;
	LODPStatus = HAL_GPIO_ReadPin(LODP_EXG_GPIO_Port, LODP_EXG_Pin);
	LODNStatus = HAL_GPIO_ReadPin(LODN_EXG_GPIO_Port, LODN_EXG_Pin);
	if (LODPStatus == GPIO_PIN_RESET && LODNStatus == GPIO_PIN_RESET)
		*leadsStatus = LEADP_CONNECTED_LEADN_CONNECTED;
	else if (LODPStatus == GPIO_PIN_RESET && LODNStatus == GPIO_PIN_SET)
		*leadsStatus = LEADP_CONNECTED_LEADN_NOTCONNECTED;
	else if (LODPStatus == GPIO_PIN_SET && LODNStatus == GPIO_PIN_RESET)
		*leadsStatus = LEADP_NOTCONNECTED_LEADN_CONNECTED;
	else *leadsStatus = LEADP_NOTCONNECTED_LEADN_NOTCONNECTED;
	exg.statusOfLeads = *leadsStatus;
}
/*-----------------------------------------------------------*/
void ECG_Filter()
{
	float input1 = exg.analogSample;
	float preInput1 = exg.tempFilterInputBuffer[0];
	float beforePreInput1 = exg.tempFilterInputBuffer[1];
	float output1;
	float preOutput1 = exg.tempFilterOutputBuffer[0];
	float beforePreOutput1 = exg.tempFilterOutputBuffer[1];
	// LPF: Fs=120sps, Fc=40Hz, Order=2
	output1= -0.6202 * preOutput1 - 0.2404 * beforePreOutput1 + 0.4652 * input1 + 0.9303 * preInput1 + 0.4652 * beforePreInput1;
	beforePreInput1 = preInput1;
	preInput1 = input1;
	beforePreOutput1 = preOutput1;
	preOutput1= output1;
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
	output2= -0.6202 * preOutput2 - 0.2404 * beforePreOutput2 + 0.4652 * input2 + 0.9303 * preInput2 + 0.4652 * beforePreInput2;
	beforePreInput2 = preInput2;
	preInput2 = input2;
	beforePreOutput2 = preOutput2;
	preOutput2= output2;
	exg.tempFilterInputBuffer[2] = preInput2;
	exg.tempFilterInputBuffer[3] = beforePreInput2;
	exg.tempFilterOutputBuffer[2] = preOutput2;
	exg.tempFilterOutputBuffer[3] = beforePreOutput2;
	exg.filteredSample = output2;
}
/*-----------------------------------------------------------*/
void EOG_Filter()
{
	float input = exg.analogSample;
	float preInput = exg.tempFilterInputBuffer[0];
	float beforePreInput = exg.tempFilterInputBuffer[1];
	float output;
	float preOutput = exg.tempFilterOutputBuffer[0];
	float beforePreOutput = exg.tempFilterOutputBuffer[1];
	// LPF: Fs=100sps, Fc=25Hz, Order=2
	output= -0.0 * preOutput - 0.1716 * beforePreOutput + 0.2929 * input + 0.5858 * preInput + 0.2929 * beforePreInput;
	beforePreInput = preInput;
	preInput = input;
	beforePreOutput = preOutput;
	preOutput= output;
	exg.tempFilterInputBuffer[0] = preInput;
	exg.tempFilterInputBuffer[1] = beforePreInput;
	exg.tempFilterOutputBuffer[0] = preOutput;
	exg.tempFilterOutputBuffer[1] = beforePreOutput;
	exg.filteredSample = output;
}
/*-----------------------------------------------------------*/
void EEG_Filter()
{
	float input = exg.analogSample;
	float preInput = exg.tempFilterInputBuffer[0];
	float beforePreInput = exg.tempFilterInputBuffer[1];
	float output;
	float preOutput = exg.tempFilterOutputBuffer[0];
	float beforePreOutput = exg.tempFilterOutputBuffer[1];
	// LPF: Fs=100sps, Fc=30Hz, Order=2
	output= -0.3695 * preOutput - 0.1958 * beforePreOutput + 0.3913 * input + 0.7827 * preInput + 0.3913 * beforePreInput;
	beforePreInput = preInput;
	preInput = input;
	beforePreOutput = preOutput;
	preOutput= output;
	exg.tempFilterInputBuffer[0] = preInput;
	exg.tempFilterInputBuffer[1] = beforePreInput;
	exg.tempFilterOutputBuffer[0] = preOutput;
	exg.tempFilterOutputBuffer[1] = beforePreOutput;
	exg.filteredSample = output;
}
/*-----------------------------------------------------------*/
void EMG_Filter()
{
	float input1 = exg.analogSample;
	float preInput1 = exg.tempFilterInputBuffer[0];
	float beforePreInput1 = exg.tempFilterInputBuffer[1];
	float output1;
	float preOutput1 = exg.tempFilterOutputBuffer[0];
	float beforePreOutput1 = exg.tempFilterOutputBuffer[1];
	// LPF: Fs=500sps, Fc=150Hz, Order=2
	output1= -0.3695 * preOutput1 - 0.1958 * beforePreOutput1 + 0.3913 * input1 + 0.7827 * preInput1 + 0.3913 * beforePreInput1;
	beforePreInput1 = preInput1;
	preInput1 = input1;
	beforePreOutput1 = preOutput1;
	preOutput1= output1;
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
	output2= 1.6475 * preOutput2 - 0.7009 * beforePreOutput2 + 0.8371 * input2 - 1.6742 * preInput2 + 0.8371 * beforePreInput2;
	beforePreInput2 = preInput2;
	preInput2 = input2;
	beforePreOutput2 = preOutput2;
	preOutput2= output2;
	exg.tempFilterInputBuffer[2] = preInput2;
	exg.tempFilterInputBuffer[3] = beforePreInput2;
	exg.tempFilterOutputBuffer[2] = preOutput2;
	exg.tempFilterOutputBuffer[3] = beforePreOutput2;
	exg.filteredSample = output2;
}
/*-----------------------------------------------------------*/
void ECG_BaselineFilter()
{
	float input = exg.filteredSample;
	float preInput = exg.tempFilterInputBuffer[4];
	float output;
	float preOutput = exg.tempFilterOutputBuffer[4];

	// HPF: Fs=120sps, Fc=7Hz, Order=1
	output= 0.6873 * preOutput + 0.8436 * input - 0.8436 * preInput;
	preInput = input;
	preOutput= output;
	exg.tempFilterInputBuffer[4] = preInput;
	exg.tempFilterOutputBuffer[4] = preOutput;
	exg.ECGBaselineFilteredSample = output;
}
/*-----------------------------------------------------------*/
void ECG_HeartRateCalculation()
{
	float input = exg.ECGBaselineFilteredSample;
	uint16_t period;
	float HR;
	float heartRateSum = 0;
	if (input >= ECG_THRESHOLD && exg.heartRateLock == 0)
	{
		exg.heartRateLock = 1;
		period = HAL_GetTick() - exg.HRCalculationLastTick;    // find time between tow beats in msec.
		HR = 60000.0 / (float)period;
		if(HR >= HEART_RATE_MIN && HR <= HEART_RATE_MAX)
		{
			// if relative change between current HR and old HR within specific range, send HR else ignore sending current value (there is noise)
			if ((HR >= 0.8 * exg.previousHeartRate) && (HR <= 1.2 * exg.previousHeartRate))
				{
				exg.heartRateArray[exg.heartRateIndex ++] = HR;
					if(exg.heartRateIndex == HEART_RATE_ARRAY_SIZE)
					{
						exg.heartRateIndex = 0;
						for (uint8_t i=0; i<HEART_RATE_ARRAY_SIZE; i++)
							heartRateSum += exg.heartRateArray[i];
						exg.heartRate = roundf (heartRateSum / HEART_RATE_ARRAY_SIZE);
					}
				}
		}
		else
		{
			HR = 0;
			exg.heartRate = HR;
			exg.heartRateIndex = 0;  // empty heartRateArray when happening wrong heart rate
		}
		exg.previousHeartRate = HR;
		exg.HRCalculationLastTick = HAL_GetTick();
	}
	else if(input < ECG_THRESHOLD)
		exg.heartRateLock = 0;
}
/*-----------------------------------------------------------*/
void EOG_EnvelopeDetection()
{
	float input = exg.EMGRectifiedSample;
	uint8_t index = exg.windowBufferIndex;
	float lastSampleInWindow = exg.movingWindowBuffer[index];
	float sum = exg.sumOfSamplesValuesInWindow;
	float movingMean;
	sum = sum - lastSampleInWindow;
	sum = sum + input;  // new sample get into window
	exg.movingWindowBuffer[index] = input;
	index ++;
	if (index == EMG_MOVING_WINDOW) // circular buffer
		index = 0;
	movingMean = (sum / EMG_MOVING_WINDOW) * EMG_EVELOPE_GAIN_FACTOR;
	exg.sumOfSamplesValuesInWindow = sum;
	exg.EMGEnvelopeSample = movingMean;
	exg.windowBufferIndex = index;
}
/*-----------------------------------------------------------*/
void EMG_Rectifying()
{
	float input = exg.filteredSample;
	float absInput = input;
	if (input < 0.0)
		absInput = - input;
	exg.EMGRectifiedSample = absInput;
}
/*-----------------------------------------------------------*/
void EMG_EnvelopeDetection()
{
	float input = exg.EMGRectifiedSample;
	uint8_t index = exg.windowBufferIndex;
	float lastSampleInWindow = exg.movingWindowBuffer[index];
	float sum = exg.sumOfSamplesValuesInWindow;
	float movingMean;
	sum = sum - lastSampleInWindow;
	sum = sum + input;  // new sample get into window
	exg.movingWindowBuffer[index] = input;
	index ++;
	if (index == EMG_MOVING_WINDOW) // circular buffer
		index = 0;
	movingMean = (sum / EMG_MOVING_WINDOW) * EMG_EVELOPE_GAIN_FACTOR;
	exg.sumOfSamplesValuesInWindow = sum;
	exg.EMGEnvelopeSample = movingMean;
	exg.windowBufferIndex = index;
}
/*-----------------------------------------------------------*/
void EMG_PulseDetection()
{
	float input = exg.EMGEnvelopeSample;
	float EMGPulseThreshold = exg.EMGPulseDetectionThreshold;
	if (input >= EMGPulseThreshold &&  exg.EMGPulseDetectionLock == 0) // detecting rising edge of pulse
	{
		exg.EMGPulseRisingEdgeTick = HAL_GetTick();
		exg.EMGPulseDetectionLock = 1;
	}
	else if (input < EMGPulseThreshold &&  exg.EMGPulseDetectionLock == 1) // detecting falling edge of pulse
	{

		uint16_t EMGPulseTime = HAL_GetTick() - exg.EMGPulseRisingEdgeTick;
		if (EMGPulseTime > EMG_NOISY_PULSE_PERIOD_MS)
		{
			exg.EMGPulseDurationMsec = EMGPulseTime;
			exg.EMGPulseDetectionFlag = 1;
		}
		exg.EMGPulseDetectionLock = 0;
	}
}
/*-----------------------------------------------------------*/
void EyeBlinkDetection()
{
	float input = exg.filteredSample;

	if (input >= EOG_BLINK_MAX_THRESHOLD   &&  exg.EOGPositivePulseDetectionLock == 0) // detecting rising edge (start) of positive pulse
			exg.EOGPositivePulseDetectionLock = 1;
	else if (input < (EOG_BLINK_MAX_THRESHOLD - SHMITH_SHIFT)  &&  exg.EOGPositivePulseDetectionLock == 1) // detecting falling edge (finish) of positive pulse
	{
		exg.EOGPositivePulseDetectionLock = 0;
		exg.EOGPositivePulseDetectionTick = HAL_GetTick();
		exg.EOGPositivePulseDetectionFlag = 1;
		if(exg.EOGNegativePulseDetectionFlag == 1)
		{
			if(exg.EOGPositivePulseDetectionTick - exg.EOGNegativePulseDetectionTick < EOG_ONE_BLINK_PERIOD_MS)
				{
					exg.eyeBlinkStatus = LEFT_BLINK;
					exg.EOGNegativePulseDetectionFlag = 0;
					exg.EOGPositivePulseDetectionFlag = 0;
				}
			else exg.EOGNegativePulseDetectionFlag = 0;
		}
	}
	else if (input <= EOG_BLINK_MIN_THRESHOLD  &&  exg.EOGNegativePulseDetectionLock == 0) //detecting falling edge (start) of negative pulse
		exg.EOGNegativePulseDetectionLock = 1;
	else if (input > (EOG_BLINK_MIN_THRESHOLD + SHMITH_SHIFT)  &&  exg.EOGNegativePulseDetectionLock == 1 ) //detecting rising edge (finish) of negative pulse
	{
		exg.EOGNegativePulseDetectionLock = 0;
		exg.EOGNegativePulseDetectionTick = HAL_GetTick();
		exg.EOGNegativePulseDetectionFlag = 1;
		if(exg.EOGPositivePulseDetectionFlag ==1)
		{
			if(exg.EOGNegativePulseDetectionTick - exg.EOGPositivePulseDetectionTick < EOG_ONE_BLINK_PERIOD_MS)
				{
					exg.eyeBlinkStatus = RIGHT_BLINK;
					exg.EOGNegativePulseDetectionFlag = 0;
					exg.EOGPositivePulseDetectionFlag = 0;
				}
			else exg.EOGPositivePulseDetectionFlag = 0;
		}
	}
}


/*-----------------------------------------------------------*/
/*  */
Module_Status EXG_SignalProcessing(void)
{
	uint8_t status = H2BR0_OK;
	LeadsStatus_EXG leadsStatus;
	InputSignal_EXG inputSignal;

	CheckLeadsStatus(&leadsStatus);

	if (leadsStatus == LEADP_CONNECTED_LEADN_CONNECTED)
	{
		exg.analogSample = (float)(exg.AdcValue) / ADC_NUM_OF_STATES * ADC_VREF; // Convert to analog: 12bit, Vref=3.3V
		inputSignal = exg.inputSignalType;

		switch (inputSignal)
		{
			case ECG:
				ECG_Filter();
				ECG_BaselineFilter();
				if(exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
					ECG_HeartRateCalculation();
				break;

			case EOG:
				EOG_Filter();
				if(exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
					EyeBlinkDetection();
				break;

			case EEG:
				EEG_Filter();
				break;

			case EMG:
				EMG_Filter();
				if(exg.sampleCounter++ > FILTER_TRANSIENT_STATE_SAMPLES)
				{
					EMG_Rectifying();
					EMG_EnvelopeDetection();
					EMG_PulseDetection();
				}
				break;

			default:
				status = H2BR0_ERR_WRONGPARAMS;
		}
	}
	else
		status = H2BR0_ERR_LEADS_NOTCONNECTED;

	return status;
}
/*-----------------------------------------------------------*/
/* timer2 EXG special timer callback */
void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef* htim)
{

	if(htim->Instance == EXG_TIM)
	{
		SetSamplingFlag();
		EXG_SignalProcessing();
	}
}

/* -----------------------------------------------------------------------
 |								  APIs							          |
/* -----------------------------------------------------------------------
 */
/*
 * @brief: Initialize the signal type to be measured.
 * @param1: inputSignal to specify signal type (EMG - ECG - EEG - EOG).
 * @retval: status
 */
Module_Status EXG_Init(InputSignal_EXG inputSignal)
{
	uint8_t status = H2BR0_OK;

	EXG_Enable();

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
			status = H2BR0_ERR_WRONGPARAMS;
			break;
	}
	Delay_ms(2000);  // avoiding transient state when module is power on
	HAL_TIM_Base_Start_IT(&HANDLER_Timer_EXG);
	HAL_ADC_Start_DMA(&HANDLER_ADC_EXG, &(exg.AdcValue), 1);

	return status;
}


/*-----------------------------------------------------------*/
/*
 * @brief: Seting the threshold for EMG signal.
 * @param1: threshold value (0 - 100).
 * @retval: status
 */
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
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: reading the time of how long the EMG signal lasted with the threshold value.
 * @param1: EMGDetectionFlag pointer to a buffer to store value.
 * @param2: EMGDurationMsec pointer to a buffer to store value.
 * @retval: status
 */
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
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: reading eye movement state (rapid right or left - up or down) based on electrode placement.
 * @param1: eyeBlinkStatus pointer to a buffer to store value.
 * @retval: status
 */
Module_Status EOG_CheckEyeBlink(EyeBlinkingStatus *eyeBlinkStatus)
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EOG)
	{
		*eyeBlinkStatus = exg.eyeBlinkStatus;
		if (*eyeBlinkStatus != NO_BLINK)
			exg.eyeBlinkStatus = NO_BLINK;
	}
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Extracting a normal sample and a filtered sample from the ECG signal.
 * @param1: sample pointer to a buffer to store value.
 * @param2: filteredSample pointer to a buffer to store value.
 * @retval: status
 */
Module_Status ECG_Sample(float *sample, float *filteredSample )
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == ECG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
	}
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Extracting a normal sample and a filtered sample from the EOG signal.
 * @param1: sample pointer to a buffer to store value.
 * @param2: filteredSample pointer to a buffer to store value.
 * @retval: status
 */
Module_Status EOG_Sample(float *sample, float *filteredSample )
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EOG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
	}
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Extracting a normal sample and a filtered sample from the EEG signal.
 * @param1: sample pointer to a buffer to store value.
 * @param2: filteredSample pointer to a buffer to store value.
 * @retval: status
 */
Module_Status EEG_Sample(float *sample, float *filteredSample )
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == EEG)
	{
		*sample = exg.analogSample;
		*filteredSample = exg.filteredSample;
	}
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Extracting a normal sample, a filtered sample, a rectified sample, and an envelope sample from the EMG signal.
 * @param1: sample pointer to a buffer to store value.
 * @param2: filteredSample pointer to a buffer to store value.
 * @param3: rectifiedSample pointer to a buffer to store value.
 * @param4: envelopeSample pointer to a buffer to store value.
 * @retval: status
 */
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
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: reading heart rate from the ECG signal.
 * @param1: heartRate pointer to a buffer to store value
 * @retval: status
 */
Module_Status ECG_HeartRate(uint8_t *heartRate)
{
	uint8_t status = H2BR0_OK;

	if (exg.inputSignalType == ECG)
		*heartRate = exg.heartRate;
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Sending (normal sample) and (filtered sample) to display on Terminal or draw
 * signals for EMG,EEG,ECG,EOG
 * @param1: The port you want to send from
 * @param2: inputSignal to specify signal type (EMG - ECG - EEG - EOG).
 * @retval: status
 */
Module_Status PlotToTerminal(uint8_t port,InputSignal_EXG inputSignal)
{
	uint8_t status = H2BR0_OK;
	uint8_t samplingFlag;
	char sendData[26];
	if(port == 0)
	return H2BR0_ERR_WRONGPARAMS;

	if(exg.inputSignalType == ECG || exg.inputSignalType == EOG || exg.inputSignalType == EEG || exg.inputSignalType == EMG)
	{
		if(exg.inputSignalType == EMG)
			sprintf(sendData, "a%5.2fb%5.2fc%5.2fd%5.2f\r\n",exg.analogSample, exg.filteredSample, exg.EMGRectifiedSample, exg.EMGEnvelopeSample);
		else
			sprintf(sendData, "a%5.2fb%5.2f\r\n", exg.analogSample, exg.filteredSample);

		GetSamplingFlag(&samplingFlag);

		if (samplingFlag == 1)
		{
			ResetSamplingFlag();
			writePxMutex(port, sendData,strlen(sendData), 100, 100);
		}
	}
	else
		status = H2BR0_ERR_WRONGPARAMS;

	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: reading Electrodes status.
 * @param1: leadsStatus pointer to a buffer to store value
 * @retval: status
 */
Module_Status LeadsStatus(LeadsStatus_EXG *leadsStatus)
{
	uint8_t status = H2BR0_OK;

	*leadsStatus = exg.statusOfLeads;
	return status;
}
/*-----------------------------------------------------------*/
/*
 * @brief: send a sample on the required port or send it to another module and
 * graduate the value on the required port.
 * @brief: if the topology file is not activated, therefore The module number is 0
 * @param1: destination module.
 * @param2: port number.
 * @param3: inputSignal to specify signal type (EMG - ECG - EEG - EOG).
 * @retval: status
 */
Module_Status SampletoPort(uint8_t module,uint8_t port, InputSignal_EXG inputSignal){
	float sample=0;
	float filteredSample=0;
	float rectifiedSample=0;
	float envelopeSample=0;
	static uint8_t temp[16]={0};
	Module_Status status =H2BR0_OK;

	if (port == 0 && module == myID) {
		return H2BR0_ERR_WrongParams;
	}
	switch (inputSignal){
	case ECG:
		status=ECG_Sample(&sample,&filteredSample);
		if(module == myID)
		{
		temp[0] = (uint8_t)((*(uint32_t *) &sample) >> 0);
		temp[1] = (uint8_t)((*(uint32_t *) &sample) >> 8);
		temp[2] = (uint8_t)((*(uint32_t *) &sample) >> 16);
		temp[3] = (uint8_t)((*(uint32_t *) &sample) >> 24);
		temp[4] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
		temp[5] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
		temp[6] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
		temp[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		writePxITMutex(port,(char* )&temp[0],8 * sizeof(uint8_t),10);
		}
		else
		{
			if (H2BR0_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
		messageParams[0] =FMT_FLOAT;
		messageParams[2] =2;
		messageParams[3] = (uint8_t)((*(uint32_t *) &sample) >> 0);
		messageParams[4] = (uint8_t)((*(uint32_t *) &sample) >> 8);
		messageParams[5] = (uint8_t)((*(uint32_t *) &sample) >> 16);
		messageParams[6] = (uint8_t)((*(uint32_t *) &sample) >> 24);
		messageParams[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
		messageParams[8] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
		messageParams[9] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
		messageParams[10] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		SendMessageToModule(module,CODE_READ_RESPONSE,(sizeof(float) * 2)+3);
		}
		break;

	case EOG:
		status=EOG_Sample(&sample,&filteredSample);
		if(module == myID)
		{
		temp[0] = (uint8_t)((*(uint32_t *) &sample) >> 0);
		temp[1] = (uint8_t)((*(uint32_t *) &sample) >> 8);
		temp[2] = (uint8_t)((*(uint32_t *) &sample) >> 16);
		temp[3] = (uint8_t)((*(uint32_t *) &sample) >> 24);
		temp[4] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
		temp[5] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
		temp[6] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
		temp[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		writePxITMutex(port,(char* )&temp[0],8 * sizeof(uint8_t),10);
		}
		else
		{
			if (H2BR0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 2;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &sample) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &sample) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &sample) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &sample) >> 24);
			messageParams[7] = (uint8_t) ((*(uint32_t*) &filteredSample) >> 0);
			messageParams[8] = (uint8_t) ((*(uint32_t*) &filteredSample) >> 8);
			messageParams[9] = (uint8_t) ((*(uint32_t*) &filteredSample) >> 16);
			messageParams[10] =(uint8_t) ((*(uint32_t*) &filteredSample) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE,(sizeof(float) * 2) + 3);
		}
		break;

	case EEG:
		status=EEG_Sample(&sample,&filteredSample);
		if(module == myID)
		{
		temp[0] = (uint8_t)((*(uint32_t *) &sample) >> 0);
		temp[1] = (uint8_t)((*(uint32_t *) &sample) >> 8);
		temp[2] = (uint8_t)((*(uint32_t *) &sample) >> 16);
		temp[3] = (uint8_t)((*(uint32_t *) &sample) >> 24);
		temp[4] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
		temp[5] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
		temp[6] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
		temp[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		writePxITMutex(port,(char* )&temp[0],8 * sizeof(uint8_t),10);
		}
		else
		{
			if (H2BR0_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
			messageParams[0] =FMT_FLOAT;
			messageParams[2] =2;
			messageParams[3] = (uint8_t)((*(uint32_t *) &sample) >> 0);
			messageParams[4] = (uint8_t)((*(uint32_t *) &sample) >> 8);
			messageParams[5] = (uint8_t)((*(uint32_t *) &sample) >> 16);
			messageParams[6] = (uint8_t)((*(uint32_t *) &sample) >> 24);
			messageParams[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
			messageParams[8] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
			messageParams[9] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
			messageParams[10] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		SendMessageToModule(module,CODE_READ_RESPONSE,(sizeof(float) * 2)+3);
		}
		break;

	case EMG:
		status=EMG_Sample(&sample,&filteredSample,&rectifiedSample,&envelopeSample);
		if(module == myID)
		{
		temp[0] = (uint8_t)((*(uint32_t *) &sample) >> 0);
		temp[1] = (uint8_t)((*(uint32_t *) &sample) >> 8);
		temp[2] = (uint8_t)((*(uint32_t *) &sample) >> 16);
		temp[3] = (uint8_t)((*(uint32_t *) &sample) >> 24);
		temp[4] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
		temp[5] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
		temp[6] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
		temp[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		temp[8] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 0);
		temp[9] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 8);
		temp[10] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 16);
		temp[11] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 24);
		temp[12] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 0);
		temp[13] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 8);
		temp[14] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 16);
		temp[15] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 24);
		writePxITMutex(port,(char* )&temp[0],16 * sizeof(uint8_t),10);
		}
		else
		{
			if (H2BR0_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
		messageParams[0] =FMT_FLOAT;
		messageParams[2] =4;
		messageParams[3] = (uint8_t)((*(uint32_t *) &sample) >> 0);
		messageParams[4] = (uint8_t)((*(uint32_t *) &sample) >> 8);
		messageParams[5] = (uint8_t)((*(uint32_t *) &sample) >> 16);
		messageParams[6] = (uint8_t)((*(uint32_t *) &sample) >> 24);
		messageParams[7] = (uint8_t)((*(uint32_t *) &filteredSample) >> 0);
		messageParams[8] = (uint8_t)((*(uint32_t *) &filteredSample) >> 8);
		messageParams[9] = (uint8_t)((*(uint32_t *) &filteredSample) >> 16);
		messageParams[10] = (uint8_t)((*(uint32_t *) &filteredSample) >> 24);
		messageParams[11] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 0);
		messageParams[12] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 8);
		messageParams[13] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 16);
		messageParams[14] = (uint8_t)((*(uint32_t *) &rectifiedSample) >> 24);
		messageParams[15] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 0);
		messageParams[16] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 8);
		messageParams[17] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 16);
		messageParams[18] = (uint8_t)((*(uint32_t *) &envelopeSample) >> 24);
		SendMessageToModule(module,CODE_READ_RESPONSE,19);
		}
		break;

	default:
		status=H2BR0_ERR_WRONGPARAMS;
		break;

	}
	memset(&temp[0],0,sizeof(temp));

	    return status;
}
/*-----------------------------------------------------------*/
/*
 * @brief: send a Stream  on the required port or send it to another module and graduate
 * the value on the required port.
 * @brief: if the topology file is not activated, therefore The module number is 0
 * @param1: destination module.
 * @param2: port number.
 * @param3: inputSignal to specify signal type (EMG - ECG - EEG - EOG).
 * @param4: number of samples to be send.
 * @param5: timeout.
 * @retval: status
 */
Module_Status StreamtoPort(uint8_t module,uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout)
{
	Module_Status status =H2BR0_OK;
	tofMode = STREAM_TO_PORT;
	port2 = port;
	module2 = module;
	Numofsamples2 = Numofsamples;
	timeout2 = timeout;
	mode2 = inputSignal;
	return status;

}
/*-----------------------------------------------------------*/
Module_Status ExportStreanToPort (uint8_t module,uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout)
{
	Module_Status status =H2BR0_OK;
	uint32_t samples=0;
	uint32_t period=0;
	period=timeout/Numofsamples;

	if (timeout < MIN_PERIOD_MS || period < MIN_PERIOD_MS)
		return H2BR0_ERR_WRONGPARAMS;

	while(samples < Numofsamples)
	{
	status=SampletoPort(module,port,inputSignal);
	vTaskDelay(pdMS_TO_TICKS(period));
	samples++;
	}
	tofMode=20;
	samples=0;
	return status;

}
/*-----------------------------------------------------------*/
Module_Status StreamToTerminal(uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout)
{
	Module_Status status =H2BR0_OK;
	tofMode = STREAM_TO_Terminal;
	port1 = port;
	Numofsamples1 = Numofsamples;
	timeout1 = timeout;
	mode1 = inputSignal;
	return status;

}

/*-----------------------------------------------------------*/
Module_Status ExportStreanToTerminal (uint8_t port,InputSignal_EXG inputSignal,uint32_t Numofsamples,uint32_t timeout)
{
	Module_Status status =H2BR0_OK;
	uint32_t samples=0;
	uint32_t period=0;
	period=timeout/Numofsamples;

	if (timeout < MIN_PERIOD_MS || period < MIN_PERIOD_MS)
		return H2BR0_ERR_WRONGPARAMS;

	while(samples < Numofsamples)
	{
	status=PlotToTerminal(port,inputSignal);
	vTaskDelay(pdMS_TO_TICKS(period));
	samples++;
	}
	tofMode=20;
	samples=0;
	return status;

}
/*-----------------------------------------------------------*/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr=1 ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[PcPort-1][chr] == '\r') {
				UARTRxBuf[PcPort-1][chr] = 0;
				flag=1;
				return H2BR0_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H2BR0_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H2BR0_OK;
}
/*-----------------------------------------------------------*/
static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H2BR0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H2BR0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r' ) {
				UARTRxBuf[PcPort - 1][chr] = 0;
			}
		}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(PcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,Numofsamples) != H2BR0_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");
	return status;
}
/*-----------------------------------------------------------*/
void SampleEMGToString(char *cstring, size_t maxLen) {
	float sample, filteredSample, rectifiedSample, envelopeSample;
	EMG_Sample(&sample, &filteredSample, &rectifiedSample, &envelopeSample);
	snprintf(cstring, maxLen,
			"Sample: %.3f | FilteredSample: %.3f | RectifiedSample: %.3f |EnvelopeSample: %.3f \r\n",
			sample, filteredSample, rectifiedSample, envelopeSample);
}
/*-----------------------------------------------------------*/
void SampleEEGToString(char *cstring, size_t maxLen) {
	float sample, filteredSample;
	EEG_Sample(&sample, &filteredSample);
	snprintf(cstring, maxLen, "sample: %.3f filteredSample: %.3f  \r\n", sample,
			filteredSample);
}
/*-----------------------------------------------------------*/
void SampleEOGToString(char *cstring, size_t maxLen) {
	float sample, filteredSample;
	EOG_Sample(&sample, &filteredSample);
	snprintf(cstring, maxLen, "sample: %.3f filteredSample: %.3f  \r\n", sample,
			filteredSample);
}
/*-----------------------------------------------------------*/
void SampleECGToString(char *cstring, size_t maxLen) {
	float sample, filteredSample;
	ECG_Sample(&sample, &filteredSample);
	snprintf(cstring, maxLen, "sample: %.3f filteredSample: %.3f  \r\n", sample,
			filteredSample);
}
/*-----------------------------------------------------------*/
Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout,
		InputSignal_EXG inputSignal) {

	switch (inputSignal) {
	case EMG:
		StreamMemsToCLI(Numofsamples, timeout, SampleEMGToString);
		break;
	case EEG:
		StreamMemsToCLI(Numofsamples, timeout, SampleEEGToString);
		break;
	case EOG:
		StreamMemsToCLI(Numofsamples, timeout, SampleEOGToString);
		break;
	case ECG:
		StreamMemsToCLI(Numofsamples, timeout, SampleECGToString);
		break;
	default:
		break;
	}

}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */

portBASE_TYPE StreamEXGCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const EMGCmdName = "emg";
	const char *const EEGCmdName = "eeg";
	const char *const EOGCmdName = "eog";
	const char *const ECGCmdName = "ecg";

	uint32_t Numofsamples = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, EMGCmdName, strlen(EMGCmdName))) {
			if (portOrCLI) {

				StreamToCLI(Numofsamples, timeout,EMG);

			} else {

				ExportStreanToPort(module, port, EMG, Numofsamples, timeout);

			}

		} else if (!strncmp(pSensName, EEGCmdName, strlen(EEGCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, EEG);

			} else {

				ExportStreanToPort(module, port, EEG, Numofsamples, timeout);
			}

		} else if (!strncmp(pSensName, EOGCmdName, strlen(EOGCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout,EOG);

			} else {

				ExportStreanToPort(module, port, EOG, Numofsamples, timeout);
			}

		} else if (!strncmp(pSensName, ECGCmdName, strlen(ECGCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout,ECG);

			} else {

				ExportStreanToPort(module, port, ECG, Numofsamples, timeout);
			}

		} else {
			snprintf((char*) pcWriteBuffer, xWriteBufferLen,
					"Invalid Arguments\r\n");
		}

		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule)
{
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	*ppSensName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
	pPeriodMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
	pTimeoutMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
	pModStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}
/*-----------------------------------------------------------*/

portBASE_TYPE CLI_ECG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	float sample=0;
	float filteredSample=0;
	static const int8_t *pcOKMessage=(int8_t* )"ECG Sample is:            %0.5f \n\rECG Filtered Sample is:   %0.5f\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);
        EXG_Init(ECG);
	 	status=ECG_Sample(&sample,&filteredSample);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,sample,filteredSample);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_EOG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	float sample=0;
	float filteredSample=0;
	static const int8_t *pcOKMessage=(int8_t* )"EOG Sample is:            %0.5f \n\r EOG Filtered Sample is:  %0.5f \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);
		EXG_Init(EOG);
	 	status=EOG_Sample(&sample,&filteredSample);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,sample,filteredSample);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_EEG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	float sample=0;
	float filteredSample=0;
	static const int8_t *pcOKMessage=(int8_t* )"EEG Sample is:            %0.5f \n\rEEG Filtered Sample is:   %0.5f\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);
		EXG_Init(EEG);
	 	status=EEG_Sample(&sample,&filteredSample);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,sample,filteredSample);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_EMG_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	float sample=0;
	float filteredSample=0;
	float rectifiedSample=0;
	float envelopeSample=0;
	static const int8_t *pcOKMessage=(int8_t* )"EMG Sample is:            %0.5f \n\rEMG Filtered Sample is:   %0.5f \n\rEMG Rectified Sample is:  %0.5f \n\rEMG Envelope Sample is   %0.5f \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);
		EXG_Init(EMG);
		status=EMG_Sample(&sample,&filteredSample,&rectifiedSample,&envelopeSample);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,sample,filteredSample,rectifiedSample,envelopeSample);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_EMG_SetThresholdCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	uint8_t threshold = 0;
	static const int8_t *pcOKMessage=(int8_t* )"EMG Signal is on at Threshold %d\n\r";
	static const int8_t *pcErrorMessage =(int8_t* )"Error Threshold!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

		pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
		threshold = (uint8_t )atol((char* )pcParameterString1);
		EXG_Init(EMG);
		status = EMG_SetThreshold(threshold);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,threshold);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_EMG_CheckPulseCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	uint8_t EMGDetectionFlag=0;
	uint16_t EMGDurationMsec=0;
	static const int8_t *pcOKMessage=(int8_t* )"EMG Detection Flag is:    %d\n\rEMG Duration Msec is:     %d\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

		EXG_Init(EMG);
	 	status=EMG_CheckPulse(&EMGDetectionFlag,&EMGDurationMsec);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,EMGDetectionFlag,EMGDurationMsec);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ECG_HeartRateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	uint8_t heartRate=0;
	static const int8_t *pcOKMessage=(int8_t* )"Heart Rate is: %d bpm\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

		EXG_Init(ECG);
	 	status=ECG_HeartRate(&heartRate);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,heartRate);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_EOG_CheckEyeBlinkCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	EyeBlinkingStatus eyeBlinkStatus;
	static const int8_t *pcOKMessage=(int8_t* )"Eye Blink Status is: %s \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";
	const char* eyeBlinkStatusStrings[] = {
	    "No blink",
	    "Right blink",
	    "Left blink"
	};
		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

		EXG_Init(EOG);
	 	status=EOG_CheckEyeBlink(&eyeBlinkStatus);

	 if(status == H2BR0_OK)
	 {
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,eyeBlinkStatusStrings[eyeBlinkStatus]);
	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_LeadsStatusCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR0_OK;
	LeadsStatus_EXG leadsStatus;
	static const int8_t *pcOKMessage=(int8_t* )"Electrodes Status is: %s \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";
	const char* electrodesStatusStrings[] = {
			"LeadP Connected LeadN Connected",
			"LeadP Connected LeadN Not Connected",
			"LeadP Not Connected LeadN Connected",
			"LeadP Not Connected LeadN Not Connected"
	};
		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=LeadsStatus(&leadsStatus);

	 if(status == H2BR0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,electrodesStatusStrings[leadsStatus]);

	 }

	 else if(status == H2BR0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}


/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
