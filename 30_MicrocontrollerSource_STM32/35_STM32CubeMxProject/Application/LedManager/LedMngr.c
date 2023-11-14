#include "main.h"
#include "LedMngr.h"
#include "EEmu.h"

#include "BluetoothClassicComm.h"

static uint32_t QuickBlinksCounter = 0;



typedef enum
{
	LED_Idle,
	LED_Active,
}LedBlinkState_t;

static LedBlinkState_t LedBlinkState;

static void UpdateLedState(void)
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
	BLU_RegisterNvMdataUpdateInfoCallBack(UpdateLedState);
}

void LED_MngrTask()
{
	static uint32_t BlinkSavedTime;
	static uint32_t QuickBlindTimer;

	if(LedBlinkState == LED_Active) // if(LedBlinkState == LED_Active)
	{
	  if(QuickBlindTimer+LED_QUICK_TOGGLE_TIME < HAL_GetTick() && QuickBlinksCounter > 0 )
	  {
		QuickBlinksCounter--;
		    QuickBlindTimer= HAL_GetTick();
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			HAL_GPIO_TogglePin(SenD1_GPIO_Port, SenD1_Pin);
			HAL_GPIO_TogglePin(SenD2_GPIO_Port, SenD2_Pin);
	  }
	  else if( BlinkSavedTime+LED_TOGGLE_TIME < HAL_GetTick() && (QuickBlinksCounter == 0))
	  {
		  BlinkSavedTime= HAL_GetTick();
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			HAL_GPIO_TogglePin(SenD1_GPIO_Port, SenD1_Pin);
			HAL_GPIO_TogglePin(SenD2_GPIO_Port, SenD2_Pin);

	  }
	}
	else
	{ /*Lazy solution :) */
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SenD1_GPIO_Port, SenD1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SenD2_GPIO_Port, SenD2_Pin, GPIO_PIN_RESET);
	}
}



void  LED_DoQuickBlinks(uint32_t qBlinksCount)
{
	QuickBlinksCounter = qBlinksCount;
}