#ifndef _Encoders_Module_H
#define _Encoders_Module_H


#include "stdint.h"
#include "Macro_Settings.h"


typedef struct
{
	uint32_t LeftEncoderImpulsCount;
	uint32_t RightEncoderImpulsCount;

	float Distance_LeftWheel;
	float Distance_RightWheel;

	int ProbeNumber;

	uint32_t PreviousLeftEncoderImpulsCount;
	uint32_t PreviousRightEncoderImpulsCount;

	float LeftWheelDistanceInProbe[MaxProbeNumber];
	float RightWheelDistanceInProbe[MaxProbeNumber];


	float LeftWheelSpeed;
	float RightWheelSpeed;

	float SpeedInProbe[MaxProbeNumber];
	float LeftWheelSpeedInProbe[MaxProbeNumber];
	float RightWheelSpeedInProbe[MaxProbeNumber];
	float TakenDistanceInProbe[MaxProbeNumber];

	float AverageSpeed;
	float TakenDistance;

}Encoders_Module_t ;

extern Encoders_Module_t Enc_Module;

void Encoder_ModuleInit();
void Enc_ResetModule();
void Enc_AddEncoderImpulsIntoImpulsSum(uint16_t GPIO_Pin);
void Enc_CalculateActualSpeed();
void Enc_CalculateTraveledDistance();
void Enc_CalculateFinalAverageSpeed();

#endif //_Encoders_Module_H
