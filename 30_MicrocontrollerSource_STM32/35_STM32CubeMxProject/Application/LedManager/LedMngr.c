#include "main.h"
#include "LedMngr.h"
#include "EEmu.h"
#include "BLE_Comm.h"



typedef enum
{
	LED_Idle,
	LED_Active,
}LedBlinkState_t;

static LedBlinkState_t LedBlinkState;

static UpdateLedState(void)
{
	uint32_t _tmpLedBlinkState;
	EE_ReadVariableU32(EE_NvmAddr_BlinkLadeState_U32,&_tmpLedBlinkState);
	LedBlinkState = (LedBlinkState_t)_tmpLedBlinkState;
}


void LED_Init()
{
	uint32_t _tmpLedBlinkState;
	EE_ReadVariableU32(EE_NvmAddr_BlinkLadeState_U32,&_tmpLedBlinkState);
	LedBlinkState = (LedBlinkState_t)_tmpLedBlinkState;
	BLE_RegisterNvMdataUpdateInfoCallBack(UpdateLedState);
}

void LED_MngrTask()
{
	static uint32_t BlinkSavedTime;

	if(LedBlinkState == LED_Active)
	{
	  if( BlinkSavedTime+LED_TOGGLE_TIME < HAL_GetTick())
	  {
		  BlinkSavedTime= HAL_GetTick();
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }
	}
	else
	{ /*Lazy solution :) */
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
}