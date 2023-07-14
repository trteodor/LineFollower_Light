/*
 * ENKODERY.c
 *
 *  Created on: Jan 11, 2021
 *      Author: Teodor
 */

#include "EncodersHandler.h"
#include "LF_AppConfig.h"
#include "BLE_Comm.h"
#include "EEmu.h"

#include "math.h"
#include "tim.h"

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

#define LeftEncoderCnt htim8.Instance->CNT
#define RightEncoderCnt htim4.Instance->CNT


/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/


typedef struct EncHandlerDescr_t
{
	uint16_t* LeftEncoderCntImpulsCount;
	uint16_t* RightEncoderCntImpulsCount;

	float Distance_LeftWheel;
	float Distance_RightWheel;
	int ProbeNumber;
	uint16_t PreviousLeftEncoderCntImpulsCount;
	uint16_t PreviousRightEncoderCntImpulsCount;
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

Encoders_Module_t EncHandlerDescr;



/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

void Enc_CalculateActualSpeed()
{
	static uint32_t EncodersSavedTime;

	if(HAL_Get100usTick() >EncodersSavedTime + EncodersProbeTime)
	{
		EncodersSavedTime= HAL_Get100usTick();

		/*uint overflow isn't problem becuse for example 2-255=3 so diffrent is 3 that's true*/

		uint16_t TmpLeftEncImpCnt= UINT16_MAX - *EncHandlerDescr.LeftEncoderCntImpulsCount;
		uint16_t TmpRightEncImpCnt= UINT16_MAX - *EncHandlerDescr.RightEncoderCntImpulsCount;

		EncHandlerDescr.Distance_LeftWheel=
			((TmpLeftEncImpCnt-EncHandlerDescr.PreviousLeftEncoderCntImpulsCount)*OneImpulsDistance);
		EncHandlerDescr.LeftWheelDistanceInProbe[EncHandlerDescr.ProbeNumber]=EncHandlerDescr.Distance_LeftWheel;

		EncHandlerDescr.Distance_RightWheel=
			((TmpRightEncImpCnt-EncHandlerDescr.PreviousRightEncoderCntImpulsCount)*OneImpulsDistance);
		EncHandlerDescr.RightWheelDistanceInProbe[EncHandlerDescr.ProbeNumber]=EncHandlerDescr.Distance_RightWheel;

		EncHandlerDescr.LeftWheelSpeed=EncHandlerDescr.Distance_LeftWheel/EncodersProbeTimeInSeconds;
		EncHandlerDescr.RightWheelSpeed=EncHandlerDescr.Distance_RightWheel/EncodersProbeTimeInSeconds;


		EncHandlerDescr.PreviousLeftEncoderCntImpulsCount=TmpLeftEncImpCnt;
		EncHandlerDescr.PreviousRightEncoderCntImpulsCount=TmpRightEncImpCnt;


		
		EncHandlerDescr.SpeedInProbe[EncHandlerDescr.ProbeNumber]=
				(EncHandlerDescr.LeftWheelSpeedInProbe[EncHandlerDescr.ProbeNumber]+EncHandlerDescr.RightWheelSpeedInProbe[EncHandlerDescr.ProbeNumber]);

		/*Divide by 0 is prohibited...  ;) */
		if(EncHandlerDescr.SpeedInProbe[EncHandlerDescr.ProbeNumber] !=0)
		{
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			EncHandlerDescr.SpeedInProbe[EncHandlerDescr.ProbeNumber]=EncHandlerDescr.SpeedInProbe[EncHandlerDescr.ProbeNumber] / 2;
		}
		else{
			/*nothing to do*/
		}


		EncHandlerDescr.TakenDistanceInProbe[EncHandlerDescr.ProbeNumber] =
				EncHandlerDescr.LeftWheelDistanceInProbe[EncHandlerDescr.ProbeNumber] +
					EncHandlerDescr.RightWheelDistanceInProbe[EncHandlerDescr.ProbeNumber];

			if(EncHandlerDescr.TakenDistanceInProbe[EncHandlerDescr.ProbeNumber] != 0)
			{
				EncHandlerDescr.TakenDistanceInProbe[EncHandlerDescr.ProbeNumber]=EncHandlerDescr.TakenDistanceInProbe[EncHandlerDescr.ProbeNumber]/2;
			}
		EncHandlerDescr.TakenDistance=EncHandlerDescr.TakenDistance + EncHandlerDescr.TakenDistanceInProbe[EncHandlerDescr.ProbeNumber];



	// for(int i=0; i < EncHandlerDescr.ProbeNumber; i++)
	// 	{
	// 		if(EncHandlerDescr.SpeedInProbe[i] != 0)
	// 		{
	// 			EncHandlerDescr.AverageSpeed=EncHandlerDescr.AverageSpeed+EncHandlerDescr.SpeedInProbe[i];
	// 		}
	// 	}
	// EncHandlerDescr.AverageSpeed=EncHandlerDescr.AverageSpeed / EncHandlerDescr.ProbeNumber;



		EncHandlerDescr.ProbeNumber++;
	}

}

static void ReadNvMParameters(void)
{
	/*Currently none NvmData to read/ update*/
}

static void BleEncUpdateNvmDataCallBack(void)
{
	ReadNvMParameters();
}

static void DataEncBleReporter(void)
{
// 	typedef struct
// {
// 	float WhLftSp;
// 	float WhRhtSp;
// 	float YawRate;
// 	float PosX;
// 	float PosY;
// 	float TravelledDistance;
// }BLE_MapDataReport_t; /*Current size 6*4 = 24*/
}

/*******************************************************************************/
void ENC_Init(void)
{
	EncHandlerDescr.LeftEncoderCntImpulsCount = (uint16_t*)&LeftEncoderCnt;
	EncHandlerDescr.RightEncoderCntImpulsCount = (uint16_t*)&RightEncoderCnt;

	LeftEncoderCnt = 0;
	RightEncoderCnt = 0;

	EncHandlerDescr.Distance_LeftWheel = 0;
	EncHandlerDescr.Distance_RightWheel = 0;
	EncHandlerDescr.ProbeNumber = 0;
	EncHandlerDescr.PreviousLeftEncoderCntImpulsCount = 0;
	EncHandlerDescr.PreviousRightEncoderCntImpulsCount = 0;
	EncHandlerDescr.LeftWheelSpeed = 0;
	EncHandlerDescr.RightWheelSpeed = 0;
	EncHandlerDescr.AverageSpeed=0;
	EncHandlerDescr.TakenDistance = 0;

	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	ReadNvMParameters();
	BLE_RegisterNvMdataUpdateInfoCallBack(BleEncUpdateNvmDataCallBack);

}

/*******************************************************************************/
void ENC_Task(void)
{

}
/*******************************************************************************/

float ENC_GetTravelledDistance(void)
{

}

/*******************************************************************************/
float ENC_GetCurrentSpeed(void)
{

}

/*******************************************************************************/
float ENC_GetAverageSpeed(void)
{

}




