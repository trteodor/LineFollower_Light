
#include "main.h"
#include "tim.h"
#include "string.h"
#include <stdio.h>
#include "stdbool.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "LF_Menager.h"
#include "LF_LinePid.h"
#include "BLE_Comm.h"
#include "EEmu.h"
#include "LedMngr.h"
#include "EncodersHandler.h"
#include "LinePosEstimator.h"

#include "stdbool.h"


#define MaxPID_DerivativeTime 200
#define MaxPWMValue 1000 //Depends on the PWM value of the timer counter (Max Timer Count Value dk how to explain :( )


#define CountStatesWhenLineBy4SenDetToEndLapMark 25

/*************************************************************************/

typedef struct PositionOnTrack_t
{
	float X[MaxProbeNumber];
	float Y[MaxProbeNumber];
	float T[MaxProbeNumber];
}PositionOnTrack_t;

typedef enum RobotState_t
{
	LF_Ok,
	LF_Idle,
	LF_go_Stop,
	LF_go_Start,
	LF_Started,
}RobotState_t;

typedef enum EndLapMarkStates_t
{
	ResetState,
	Start_MarkDet,
	End_MarkDet,

}EndLapMarkStates_t;

typedef enum TrackMapActions_t
{
	MapSt_Idle,
	MapSt_GoToCreate,
	MapSt_Created,
}TrackMapActions_t;

typedef enum TryDetectEndLapMarkState_t
{
	NotActive,
	Active,
}TryDetectEndLapMarkState_t;


typedef struct Robot_Cntrl_t
{
	EndLapMarkStates_t EndLapMarkStates;
	TryDetectEndLapMarkState_t TryDetEndLapMarkState;
	RobotState_t RobotState;
	TrackMapActions_t TrackMapActions;
	bool EndLapMarkDetection;
	bool IsMapAvailable;
	bool IsFlagStartedForDrivingTime;

	float RobotStartTime;
	float RobotStopTime;
	float RobotRunTime;

	uint32_t SavedCountEncProbeNumerWhenRStopped;

}Robot_Cntrl_t;

/*************************************************************************/

Robot_Cntrl_t Robot_Cntrl;
// extern PID_RegModule_t PID_Module;
PositionOnTrack_t PositionOnTrack;

static void Decode_PID();
static void LF_Robot_Stop();
static void Motor_PWM_Init();
static void ForwardDriving(int LeftMotorSpeed, int RightMotorSpeed);
static void RightMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void LeftMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void App_RobotControllerInit();
static RobotState_t TryDetectLapEndMark();
static RobotState_t App_RobotControllerTask();
static void App_SpeedProfiler();
static void CalculateDrivingTimeFun();


/**************************************************************************************************/
void Motor_PWM_Init()
{
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,0);
    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,0);

    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
}
/**************************************************************************************************/
static void ForwardDriving(int LeftMotorSpeed, int RightMotorSpeed)
{
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue  - RightMotorSpeed);//-->> Forward
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);

  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue - LeftMotorSpeed );  //-->> Forward
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
}
/**************************************************************************************************/
static void RightMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{

	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue );
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue - RightMotorSpeed);  //-->> Reverse

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue  - LeftMotorSpeed);  //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
}
/**************************************************************************************************/
static void LeftMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue - RightMotorSpeed); //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue );

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue - LeftMotorSpeed);  //-->> Reverse

}
/**************************************************************************************************/
static void App_RobotControllerInit()
{
	// EEPROM_ReadTryDetectEndLineMarkState();
	Motor_PWM_Init();
}
/**************************************************************************************************/

static RobotState_t App_RobotControllerTask()
{

	if( Robot_Cntrl.TryDetEndLapMarkState==Active && Robot_Cntrl.EndLapMarkDetection==true )
	{

			if(Robot_Cntrl.EndLapMarkStates==ResetState)
			{
				Robot_Cntrl.EndLapMarkStates=Start_MarkDet;
				Robot_Cntrl.EndLapMarkDetection=false;
			}
			if(Robot_Cntrl.EndLapMarkStates==Start_MarkDet)
			{
				Robot_Cntrl.EndLapMarkStates=End_MarkDet;
				Robot_Cntrl.EndLapMarkDetection=false;

				LF_Robot_Stop();

				if(Robot_Cntrl.IsFlagStartedForDrivingTime==true)
				{
					Robot_Cntrl.IsFlagStartedForDrivingTime=false;
					CalculateDrivingTimeFun();
				}

				Robot_Cntrl.RobotState = LF_Idle;
			}
	}

	if(Robot_Cntrl.RobotState == LF_go_Stop) //Bluetooth or Ir Rec Can set the state
	{
		LF_Robot_Stop();
		if(Robot_Cntrl.IsFlagStartedForDrivingTime==true)
		{
			Robot_Cntrl.IsFlagStartedForDrivingTime=false;
			CalculateDrivingTimeFun();
		}
		Robot_Cntrl.RobotState = LF_Idle;
	}

	if(Robot_Cntrl.RobotState == LF_go_Start) //Bluetooth or Ir Rec Can set the state
	{
		//All falgs/values to reset state...
		Robot_Cntrl.EndLapMarkStates=ResetState;
		Robot_Cntrl.IsMapAvailable=false;
		Robot_Cntrl.IsFlagStartedForDrivingTime=true;
		Enc_ResetModule(); //reset Encoder module to zero (all struct fields to zero)
		Robot_Cntrl.RobotStartTime=HAL_GetTick(); //save start time..
		Robot_Cntrl.RobotState = LF_Started;
	}

	if(Robot_Cntrl.RobotState == LF_Started)
	{
		Decode_PID();
		TryDetectLapEndMark();

		return LF_Started;
	}
return LF_Idle;
}
/**************************************************************************************************/
static void App_SpeedProfiler()
{
	static uint8_t speedprofilenumer=0;

	if (speedprofilenumer==1)
	{
//		if(Enc_Module.TakenDistance > 0)
//			{
//			//actions
////			PID_Module.BaseMotorSpeed=1.0;
//			}
		//actions
	}
}
/******************************************************************************************************/
/******************************************************************************************************/

/* @brief
 * Equations for determining the position of the robot based on the collected information.
 * ref: https://github.com/trteodor/FAST_Line_Follower_STM32H7/blob/master/docs/LF_Desc_PL.pdf
 * equal number 3.12
 */
void Create_XY_PositionMap()
{
	// for(int i=0;  i<Enc_Module.ProbeNumber; i++)
	// {
	// 	PositionOnTrack.T[i]=PositionOnTrack.T[i-1] + (1/0.147)
	// 			*(Enc_Module.LeftWheelDistanceInProbe[i] - Enc_Module.RightWheelDistanceInProbe[i]);


	// 	PositionOnTrack.X[i]=PositionOnTrack.X[i-1]+(  0.5*cos(PositionOnTrack.T[i-1]) *
	// 			( Enc_Module.LeftWheelDistanceInProbe[i]+Enc_Module.RightWheelDistanceInProbe[i]) );

	// 	PositionOnTrack.Y[i]=PositionOnTrack.Y[i-1]+(  0.5*sin(PositionOnTrack.T[i-1]) *
	// 			( Enc_Module.LeftWheelDistanceInProbe[i]+Enc_Module.RightWheelDistanceInProbe[i]) );
	// }
}

/******************************************************************************************************/
static void CalculateDrivingTimeFun()
{
	//Calculate driving time
		Robot_Cntrl.RobotStopTime=HAL_GetTick();
		Robot_Cntrl.RobotRunTime=(Robot_Cntrl.RobotStopTime-Robot_Cntrl.RobotStartTime)/1000; //1000 to seconds
		Enc_CalculateTraveledDistance();
		Enc_CalculateFinalAverageSpeed();
		// Robot_Cntrl.SavedCountEncProbeNumerWhenRStopped=Enc_Module.ProbeNumber;
		Robot_Cntrl.IsMapAvailable=true;

		// BLE_App.Ble_AppSt=SendDrivingTimeAndAvSpeed; //just flag for BleTask
}

static void LF_Robot_Stop()
{
	//Motors Off
    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);
    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue);

    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);

}

/******************************************************************************************************/
static RobotState_t TryDetectLapEndMark()
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
	return LF_Ok;
}

/******************************************************************************************************/
static void Decode_PID()
{
	// if(PID_Module.CalculatedLeftMotorSpeed<0)
	// 	{

	// 		int _CalculatedLeftMotorSpeed=PID_Module.CalculatedLeftMotorSpeed*(-1);

	// 		LeftMotorDrivingReverse(_CalculatedLeftMotorSpeed, PID_Module.CalculatedRightMotorSpeed);
	// 		return;
	// 	}
	// if(PID_Module.CalculatedRightMotorSpeed < 0)
	// 	{
	// 	int _CalculatedRightMotorSpeed=PID_Module.CalculatedRightMotorSpeed*(-1);

	// 		RightMotorDrivingReverse(PID_Module.CalculatedLeftMotorSpeed, _CalculatedRightMotorSpeed);

	// 		return;
	// 	}

	// ForwardDriving(PID_Module.CalculatedLeftMotorSpeed, PID_Module.CalculatedRightMotorSpeed);
	return;
}




/************************************************************************************/
#define API_FUNCTIONS
/************************************************************************************/
void LF_MngrInit(void) /*Line Following Menager init */
{

}
void LF_MngrTask(void) /*Line Following Menager task */
{

}

