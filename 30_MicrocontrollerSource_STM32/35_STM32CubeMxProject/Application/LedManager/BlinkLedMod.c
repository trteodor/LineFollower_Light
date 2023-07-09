#include "main.h"
#include "BlinkLedMod.h"
#include "EEmuConfig.h"

static void EEPROM_LED_BLINK_READ();

LedBlinkState_t LedBlinkState;

void BlinkLedInit()
{
	EEPROM_LED_BLINK_READ();
}

void BlinkLedTask()
{
	static uint32_t BlinkSavedTime;


	if(LedBlinkState == LED_Active)
	{

	  if( BlinkSavedTime+LED_TOGGLE_TIME < HAL_GetTick())
	  {
		  BlinkSavedTime= HAL_GetTick();

//			  HAL_GPIO_WritePin(LDD1_GPIO_Port, LDD1_Pin,GPIO_PIN_SET);
//			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
			  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }
	}
}


static void EEPROM_LED_BLINK_READ()
{
	uint32_t tmpLedBlinkState;
//	EEPROM_READ_INT(EEPROM_LedModeState_Addr, (int *)&tmpLedBlinkState);
	LedBlinkState = tmpLedBlinkState;
}
