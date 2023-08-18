/*
 * ENKODERY.c
 *
 *  Created on: Jan 11, 2021
 *      Author: Teodor
 */
#include "stm32h7xx_it.h"

#include "EncodersHandler.h"
#include "LF_AppConfig.h"
#include "EEmu.h"

#include "math.h"
#include "tim.h"
#include "../BluetoothComm/BluetoothClassicComm.h"

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

#define LeftEncoderCnt htim8.Instance->CNT
#define RightEncoderCnt htim4.Instance->CNT


/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

typedef struct PositionStruct_t
{
	float X; 
	float Y;
	float O; /*Orientation / angle */
	float prX;
	float prY;
	float prO;
}PositionStruct_t;


typedef struct EncHandlerDescr_t
{
	uint16_t* LeftEncoderCntImpulsCount;
	uint16_t* RightEncoderCntImpulsCount;

	float Distance_LeftWheel;
	float Distance_RightWheel;
	uint32_t ProbeCounter;
	uint16_t PreviousLeftEncoderCntImpulsCount;
	uint16_t PreviousRightEncoderCntImpulsCount;

	float NVM_WheelBase;
	float NVM_OneImpulsDistance;


	float LeftWheelSpeed;
	float RightWheelSpeed;

	float LineFollowerSpeed;

	float TakenDistance;
	float AverageSpeed;

}Encoders_Module_t ;

Encoders_Module_t EncHandlerDescr = {.ProbeCounter = 1};
PositionStruct_t RobotPosition = {0};

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/* @brief
 * Equations for determining the position and orientation of the robot based on the collected information.
 * ref: https://github.com/trteodor/FAST_Line_Follower_STM32H7/blob/master/docs/LF_Desc_PL.pdf
 * equasition, page 21: 3.12
 */
static void EsimtateRobotOrientationAndPosition(void)
{
	RobotPosition.O=RobotPosition.prO +
			( (1/EncHandlerDescr.NVM_WheelBase) *(EncHandlerDescr.Distance_LeftWheel - EncHandlerDescr.Distance_RightWheel) );

	RobotPosition.X=RobotPosition.prX+(  0.5*cos(RobotPosition.prO) *
			( EncHandlerDescr.Distance_LeftWheel + EncHandlerDescr.Distance_RightWheel ) );

	RobotPosition.Y=RobotPosition.prY+ (  0.5*sin(RobotPosition.prO) *
			           ( EncHandlerDescr.Distance_LeftWheel + EncHandlerDescr.Distance_RightWheel ) );

	RobotPosition.prX = RobotPosition.X;
	RobotPosition.prY = RobotPosition.Y;
	RobotPosition.prO = RobotPosition.O;
}

static void CalculateBasicBasedOnEncodersData(void)
{
	EncHandlerDescr.ProbeCounter++;

	/*uint overflow isn't problem becuse for example 2-255=3 so diffrent is 3 that's true*/
	uint16_t TmpLeftEncImpCnt= UINT16_MAX - *EncHandlerDescr.LeftEncoderCntImpulsCount;
	uint16_t TmpRightEncImpCnt= UINT16_MAX - *EncHandlerDescr.RightEncoderCntImpulsCount;


	int16_t DifferenceValueLeft;
	int16_t DifferenceValueRight;

	DifferenceValueLeft = TmpLeftEncImpCnt-EncHandlerDescr.PreviousLeftEncoderCntImpulsCount;
	DifferenceValueRight = TmpRightEncImpCnt-EncHandlerDescr.PreviousRightEncoderCntImpulsCount;

	EncHandlerDescr.Distance_LeftWheel=((DifferenceValueLeft)*EncHandlerDescr.NVM_OneImpulsDistance);

	EncHandlerDescr.Distance_RightWheel = ((DifferenceValueRight)*EncHandlerDescr.NVM_OneImpulsDistance);

	EncHandlerDescr.LeftWheelSpeed = EncHandlerDescr.Distance_LeftWheel/EncodersProbeTimeInSeconds;
	EncHandlerDescr.RightWheelSpeed = EncHandlerDescr.Distance_RightWheel/EncodersProbeTimeInSeconds;

	/*Estimate LineFollower speed*/
	EncHandlerDescr.LineFollowerSpeed= (EncHandlerDescr.LeftWheelSpeed + EncHandlerDescr.RightWheelSpeed) / 2;

	/*Integral of speed is equal distance*/
	EncHandlerDescr.TakenDistance += EncHandlerDescr.LineFollowerSpeed /2;


	/*Estimage Average speed  Xav[n] = Xav[n-1] + (1/n)*(Zn-Xav[n-1])   */
	EncHandlerDescr.AverageSpeed += EncHandlerDescr.AverageSpeed +
									 ( (1/EncHandlerDescr.ProbeCounter) *
									  	   (EncHandlerDescr.LineFollowerSpeed - EncHandlerDescr.AverageSpeed) );

	/* Save current data as previos (input handled)*/
	EncHandlerDescr.PreviousLeftEncoderCntImpulsCount=TmpLeftEncImpCnt;
	EncHandlerDescr.PreviousRightEncoderCntImpulsCount=TmpRightEncImpCnt;

}






static void ReadNvMParameters(void)
{
	EE_ReadVariableF32(EE_NvmAddr_EncodersOneImpDistance_F32, &EncHandlerDescr.NVM_OneImpulsDistance);
	EE_ReadVariableF32(EE_NvmAddr_EncodersWheelBaseInfo_F32, &EncHandlerDescr.NVM_WheelBase);
}

static void BleEncUpdateNvmDataCallBack(void)
{
	ReadNvMParameters();
}

static void UpdateEncBleDataReport(void)
{

	BLU_MapDataReport_t BLE_MapDataReport = {0};

	BLE_MapDataReport.WhLftSp = EncHandlerDescr.LeftWheelSpeed;
	BLE_MapDataReport.WhRhtSp = EncHandlerDescr.RightWheelSpeed;
	BLE_MapDataReport.PosX = RobotPosition.X;
	BLE_MapDataReport.PosY = RobotPosition.Y;
	BLE_MapDataReport.PosO = RobotPosition.O;
	BLE_MapDataReport.TravelledDistance = EncHandlerDescr.TakenDistance;
	// BLE_MapDataReport.YawRate =

	BLU_ReportMapData(&BLE_MapDataReport);
}

/******************************************************************************************************/
static void DetectLapEndMark(void)
{

	// int CountSensorLineDetected=0;

	// static float PreviousDistance_LeftWheel;
	// static float PreviousDistance_RightWheel;

	// static uint16_t DistanceTakenInLineDetectStateByLeftWheel;
	// static uint16_t DistanceTakenInLineDetectStateByRightWheel;

		//Sensor from 2 to 6 if is line detected...
	// for(int i=2; i<6; i++)
	// {
	// 	if ( SensorModule.SensorADCValues[i] > LineIsDetectedDefValue )
	// 	{
	// 		CountSensorLineDetected++;
	// 	}
	// 	if(CountSensorLineDetected >=4)
	// 	{
	// 		SensorModule.PositionErrorValue=0;
	// 	}

	// }

	// if(CountSensorLineDetected < 4)
	// {
	// 	DistanceTakenInLineDetectStateByRightWheel=0;
	// 	DistanceTakenInLineDetectStateByLeftWheel=0;

	// 	return LF_Ok; //@@@@@@@@@@//break, end mark not detected!!
	// }


	// if( CountSensorLineDetected >=4 && (Enc_Module.Distance_LeftWheel-PreviousDistance_LeftWheel) !=0 )
	// {
	// 	PreviousDistance_LeftWheel=Enc_Module.Distance_LeftWheel;
	// 	DistanceTakenInLineDetectStateByLeftWheel++;
	// }
	// if( CountSensorLineDetected >=4 && (Enc_Module.Distance_RightWheel - PreviousDistance_RightWheel) !=0 )
	// {
	// 	PreviousDistance_RightWheel=Enc_Module.Distance_RightWheel;
	// 	DistanceTakenInLineDetectStateByRightWheel++;
	// }


	// if(DistanceTakenInLineDetectStateByRightWheel>=CountStatesWhenLineBy4SenDetToEndLapMark
	// 		&& DistanceTakenInLineDetectStateByLeftWheel>=CountStatesWhenLineBy4SenDetToEndLapMark)
	// {
	// 	//End Line Mark Detected

	// 	DistanceTakenInLineDetectStateByRightWheel=0;
	// 	DistanceTakenInLineDetectStateByLeftWheel=0;


	// 	Robot_Cntrl.EndLapMarkDetection=true;
	// 	HAL_GPIO_TogglePin(LDD1_GPIO_Port, LDD1_Pin); //now only for tests, does it work correctly?

	// }
}


/*******************************************************************************/
void ENC_Init(void)
{
	HAL_TIM_Encoder_Stop(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim8,TIM_CHANNEL_ALL);
	LeftEncoderCnt = 0;
	RightEncoderCnt = 0;
	EncHandlerDescr.LeftEncoderCntImpulsCount = (uint16_t*)&LeftEncoderCnt;
	EncHandlerDescr.RightEncoderCntImpulsCount = (uint16_t*)&RightEncoderCnt;
	EncHandlerDescr.Distance_LeftWheel = 0;
	EncHandlerDescr.Distance_RightWheel = 0;
	EncHandlerDescr.ProbeCounter = 0;
	EncHandlerDescr.PreviousLeftEncoderCntImpulsCount = 0;
	EncHandlerDescr.PreviousRightEncoderCntImpulsCount = 0;
	EncHandlerDescr.LeftWheelSpeed = 0;
	EncHandlerDescr.RightWheelSpeed = 0;
	EncHandlerDescr.LineFollowerSpeed =0;
	EncHandlerDescr.TakenDistance = 0;
	EncHandlerDescr.AverageSpeed=0;

	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	ReadNvMParameters();
	BLU_RegisterNvMdataUpdateInfoCallBack(BleEncUpdateNvmDataCallBack);

}

/*******************************************************************************/
void ENC_Task(void)
{
	static uint32_t EncodersSavedTime;

	if(HAL_GetTick100us() >EncodersSavedTime + EncodersProbeTime)
	{
		EncodersSavedTime= HAL_GetTick100us();


		CalculateBasicBasedOnEncodersData();
		EsimtateRobotOrientationAndPosition();
		UpdateEncBleDataReport();
	}

	DetectLapEndMark();
}

/*******************************************************************************/
float ENC_GetLeftWhSpeed(void)
{
	return EncHandlerDescr.LeftWheelSpeed;
}

float ENC_GetCurrentOrientation(void)
{
	return RobotPosition.O;
}

// float ENC_GetRightWhSpeed(void)
// {
// 	return EncHandlerDescr.RightWheelSpeed;
// }

float ENC_GetTravelledDistance(void)
{
	return EncHandlerDescr.TakenDistance;
}

