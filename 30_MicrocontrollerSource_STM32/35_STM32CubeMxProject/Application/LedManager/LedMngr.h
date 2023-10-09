#ifndef INC_BlinkLedMod_H_
#define INC_BlinkLedMod_H_

#include "LF_AppConfig.h"


void LED_Init();
void LED_MngrTask();

void  LED_DoQuickBlinks(uint32_t qBlinksCount);

#endif //INC_BlinkLedMod_H_
