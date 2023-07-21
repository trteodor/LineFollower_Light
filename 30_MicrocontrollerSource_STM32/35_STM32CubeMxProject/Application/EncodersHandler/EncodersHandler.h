#ifndef _Encoders_Module_H
#define _Encoders_Module_H


#include "stdint.h"



/** @brief ENC_Init
 * @details init or re-init Encoder Hanlder module
*/
void ENC_Init(void);

/** @brief ENC_Task
 * @details Main Encoder Hanlder runnable
*/
void ENC_Task(void);


float ENC_GetTravelledDistance(void);

float ENC_GetCurrentOrientation(void);

#endif //_Encoders_Module_H
