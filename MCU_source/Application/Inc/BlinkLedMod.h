#ifndef INC_BlinkLedMod_H_
#define INC_BlinkLedMod_H_

#include "Macro_Settings.h"

typedef enum
{
	LED_Idle,
	LED_Active,
}LedBlinkState_t;

extern LedBlinkState_t LedBlinkState;

void BlinkLedInit();
void BlinkLedTask();

#endif //INC_BlinkLedMod_H_
