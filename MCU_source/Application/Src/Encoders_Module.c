/*
 * ENKODERY.c
 *
 *  Created on: Jan 11, 2021
 *      Author: Teodor
 */
#include "Encoders_Module.h"
#include "math.h"
#include "main.h"

Encoders_Module_t Enc_Module;
extern uint32_t  us100Timer;




__inline void Enc_AddEncoderImpulsIntoImpulsSum(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==ENK1_Pin){
		Enc_Module.RightEncoderImpulsCount++;
	}
	if(GPIO_Pin==ENK2_Pin)
	{
		Enc_Module.RightEncoderImpulsCount++;
	}
	if( GPIO_Pin==ENK3_Pin){
		Enc_Module.LeftEncoderImpulsCount++;
	}
	if(GPIO_Pin==ENK4_Pin)
	{
		Enc_Module.LeftEncoderImpulsCount++;
	}
}

void Enc_ResetModule()
{
	Enc_Module.LeftEncoderImpulsCount = 0;
	Enc_Module.RightEncoderImpulsCount = 0;
	Enc_Module.Distance_LeftWheel = 0;
	Enc_Module.Distance_RightWheel = 0;
	Enc_Module.ProbeNumber = 0;
	Enc_Module.PreviousLeftEncoderImpulsCount = 0;
	Enc_Module.PreviousRightEncoderImpulsCount = 0;
	Enc_Module.LeftWheelSpeed = 0;
	Enc_Module.RightWheelSpeed = 0;
	Enc_Module.AverageSpeed=0;
	Enc_Module.TakenDistance = 0;
}

void Enc_CalculateActualSpeed()
{
	static uint32_t EncodersSavedTime;

	if(us100Timer>EncodersSavedTime + EncodersProbeTime)
	{
		EncodersSavedTime=us100Timer;

		Enc_Module.Distance_LeftWheel=
			((Enc_Module.LeftEncoderImpulsCount-Enc_Module.PreviousLeftEncoderImpulsCount)*OneImpulsDistance);
		Enc_Module.LeftWheelDistanceInProbe[Enc_Module.ProbeNumber]=Enc_Module.Distance_LeftWheel;

		Enc_Module.Distance_RightWheel=
			((Enc_Module.RightEncoderImpulsCount-Enc_Module.PreviousRightEncoderImpulsCount)*OneImpulsDistance);
		Enc_Module.RightWheelDistanceInProbe[Enc_Module.ProbeNumber]=Enc_Module.Distance_RightWheel;

		Enc_Module.LeftWheelSpeed=Enc_Module.Distance_LeftWheel/EncodersProbeTimeInSeconds;
		Enc_Module.LeftWheelSpeedInProbe[Enc_Module.ProbeNumber]=Enc_Module.LeftWheelSpeed;

		Enc_Module.RightWheelSpeed=Enc_Module.Distance_RightWheel/EncodersProbeTimeInSeconds;
		Enc_Module.RightWheelSpeedInProbe[Enc_Module.ProbeNumber]=Enc_Module.RightWheelSpeed;

		Enc_Module.PreviousLeftEncoderImpulsCount=Enc_Module.LeftEncoderImpulsCount;
		Enc_Module.PreviousRightEncoderImpulsCount=Enc_Module.RightEncoderImpulsCount;

		if(Enc_Module.ProbeNumber > MaxProbeNumber)
		{
			return;
		}

		Enc_Module.SpeedInProbe[Enc_Module.ProbeNumber]=
				(Enc_Module.LeftWheelSpeedInProbe[Enc_Module.ProbeNumber]+Enc_Module.RightWheelSpeedInProbe[Enc_Module.ProbeNumber]);

		if(Enc_Module.SpeedInProbe[Enc_Module.ProbeNumber] !=0)
		{
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			Enc_Module.SpeedInProbe[Enc_Module.ProbeNumber]=Enc_Module.SpeedInProbe[Enc_Module.ProbeNumber] / 2;
		}
		Enc_CalculateTraveledDistance();
		Enc_Module.ProbeNumber++;
	}

	return;
}

void Enc_CalculateFinalAverageSpeed()
{
	for(int i=0; i < Enc_Module.ProbeNumber; i++)
		{
			if(Enc_Module.SpeedInProbe[i] != 0)
			{
				Enc_Module.AverageSpeed=Enc_Module.AverageSpeed+Enc_Module.SpeedInProbe[i];
			}
		}
	Enc_Module.AverageSpeed=Enc_Module.AverageSpeed / Enc_Module.ProbeNumber;
}

void Enc_CalculateTraveledDistance()
{
		Enc_Module.TakenDistanceInProbe[Enc_Module.ProbeNumber] =
				Enc_Module.LeftWheelDistanceInProbe[Enc_Module.ProbeNumber] +
					Enc_Module.RightWheelDistanceInProbe[Enc_Module.ProbeNumber];

			if(Enc_Module.TakenDistanceInProbe[Enc_Module.ProbeNumber] != 0)
			{
				Enc_Module.TakenDistanceInProbe[Enc_Module.ProbeNumber]=Enc_Module.TakenDistanceInProbe[Enc_Module.ProbeNumber]/2;
			}
	Enc_Module.TakenDistance=Enc_Module.TakenDistance + Enc_Module.TakenDistanceInProbe[Enc_Module.ProbeNumber];

}






