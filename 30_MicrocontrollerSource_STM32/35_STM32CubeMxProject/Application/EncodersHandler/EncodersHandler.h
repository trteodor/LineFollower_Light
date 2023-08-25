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

float ENC_GetLeftWhSpeed(void);
float ENC_GetRightWhSpeed(void);

float ENC_GetYawRateWhBased(void);

float ENC_GetAverageSpeed(void);

#endif //_Encoders_Module_H
