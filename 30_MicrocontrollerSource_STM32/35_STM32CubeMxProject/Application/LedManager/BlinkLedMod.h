#ifndef INC_BlinkLedMod_H_
#define INC_BlinkLedMod_H_

#include "LF_AppConfig.h"

typedef enum
{
	LED_Idle,
	LED_Active,
}LedBlinkState_t;

extern LedBlinkState_t LedBlinkState;

void BlinkLedInit();
void BlinkLedTask();

#endif //INC_BlinkLedMod_H_
