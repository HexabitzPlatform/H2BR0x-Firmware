/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/

/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void){

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for(;;){
	}
}
float h[100], g[100] ,t[100],y[100];
/***************************************************************************/
/* User Task */
void UserTask(void *argument){
	EXG_Init(EMG);
	/* put your code here, to run repeatedly. */
	while(1){
		EMG_Sample(h, g,t,y);
//		EEG_Sample(y, t);
	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
