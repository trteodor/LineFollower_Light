#include "main.h"
#include "tim.h"
#include "string.h"
#include <stdio.h>
#include "stdbool.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "LF_AppMain.h"
#include "PID_Reg_Module.h"
#include "BLE_Comm.h"
#include "Nvm_EEPROM.h"
#include "Nvm_EEPROM_MemMap.h"
#include "BlinkLedMod.h"
#include "Encoders_Module.h"
#include "LinePosEstimator.h"



Robot_Cntrl_t Robot_Cntrl;
extern PID_RegModule_t PID_Module;
PositionOnTrack_t PositionOnTrack;


static void Decode_PID();
static void LF_Robot_Stop();
static void Motor_PWM_Init();
static void ForwardDriving(int LeftMotorSpeed, int RightMotorSpeed);
static void RightMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void LeftMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void LF_Robot_ControlInit();
static RobotState_t TryDetectLapEndMark();
static RobotState_t LF_Robot_ControlTask();
static void EEPROM_ReadTryDetectEndLineMarkState();
static void Speed_ProfileFunction();
static void CalculateDrivingTimeFun();

void LF_App_MainConfig(void)
{
	EEPROM_WriteEnable();
	BlinkLedInit();
	BLE_Init();
	Encoder_ModuleInit();
	SM_Init();
	PID_Init();

	//Functions definded in this file
	LF_Robot_ControlInit();
}

void LF_App_MainTask(void) //only one Task without any RTOS, all works fine -- for now the solution is inaf :)
{
	BLE_Task();
	Speed_ProfileFunction();
	PID_Task();
	BlinkLedTask();

	//Functions definded in this file
	LF_Robot_ControlTask();

}


static void LF_Robot_ControlInit()
{
	EEPROM_ReadTryDetectEndLineMarkState();
	Motor_PWM_Init();
}

static RobotState_t LF_Robot_ControlTask()
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


static void Speed_ProfileFunction()
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


/* @brief
 * Equations for determining the position of the robot based on the collected information.
 * ref: https://github.com/trteodor/FAST_Line_Follower_STM32H7/blob/master/docs/LF_Desc_PL.pdf
 * equal number 3.12
 */

void Create_XY_PositionMap()
{
	for(int i=0;  i<Enc_Module.ProbeNumber; i++)
	{
		PositionOnTrack.T[i]=PositionOnTrack.T[i-1] + (1/0.147)
				*(Enc_Module.LeftWheelDistanceInProbe[i] - Enc_Module.RightWheelDistanceInProbe[i]);


		PositionOnTrack.X[i]=PositionOnTrack.X[i-1]+(  0.5*cos(PositionOnTrack.T[i-1]) *
				( Enc_Module.LeftWheelDistanceInProbe[i]+Enc_Module.RightWheelDistanceInProbe[i]) );

		PositionOnTrack.Y[i]=PositionOnTrack.Y[i-1]+(  0.5*sin(PositionOnTrack.T[i-1]) *
				( Enc_Module.LeftWheelDistanceInProbe[i]+Enc_Module.RightWheelDistanceInProbe[i]) );
	}
}

static void CalculateDrivingTimeFun()
{
	//Calculate driving time
		Robot_Cntrl.RobotStopTime=HAL_GetTick();
		Robot_Cntrl.RobotRunTime=(Robot_Cntrl.RobotStopTime-Robot_Cntrl.RobotStartTime)/1000; //1000 to seconds
		Enc_CalculateTraveledDistance();
		Enc_CalculateFinalAverageSpeed();
		Robot_Cntrl.SavedCountEncProbeNumerWhenRStopped=Enc_Module.ProbeNumber;
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

static RobotState_t TryDetectLapEndMark()
{

	int CountSensorLineDetected=0;

	static float PreviousDistance_LeftWheel;
	static float PreviousDistance_RightWheel;

	static uint16_t DistanceTakenInLineDetectStateByLeftWheel;
	static uint16_t DistanceTakenInLineDetectStateByRightWheel;

		//Sensor from 2 to 6 if is line detected...
	for(int i=2; i<6; i++)
	{
		if ( SensorModule.SensorADCValues[i] > LineIsDetectedDefValue )
		{
			CountSensorLineDetected++;
		}
		if(CountSensorLineDetected >=4)
		{
			SensorModule.PositionErrorValue=0;
		}

	}

	if(CountSensorLineDetected < 4)
	{
		DistanceTakenInLineDetectStateByRightWheel=0;
		DistanceTakenInLineDetectStateByLeftWheel=0;

		return LF_Ok; //@@@@@@@@@@//break, end mark not detected!!
	}


	if( CountSensorLineDetected >=4 && (Enc_Module.Distance_LeftWheel-PreviousDistance_LeftWheel) !=0 )
	{
		PreviousDistance_LeftWheel=Enc_Module.Distance_LeftWheel;
		DistanceTakenInLineDetectStateByLeftWheel++;
	}
	if( CountSensorLineDetected >=4 && (Enc_Module.Distance_RightWheel - PreviousDistance_RightWheel) !=0 )
	{
		PreviousDistance_RightWheel=Enc_Module.Distance_RightWheel;
		DistanceTakenInLineDetectStateByRightWheel++;
	}


	if(DistanceTakenInLineDetectStateByRightWheel>=CountStatesWhenLineBy4SenDetToEndLapMark
			&& DistanceTakenInLineDetectStateByLeftWheel>=CountStatesWhenLineBy4SenDetToEndLapMark)
	{
		//End Line Mark Detected

		DistanceTakenInLineDetectStateByRightWheel=0;
		DistanceTakenInLineDetectStateByLeftWheel=0;


		Robot_Cntrl.EndLapMarkDetection=true;
		HAL_GPIO_TogglePin(LDD1_GPIO_Port, LDD1_Pin); //now only for tests, does it work correctly?

	}
	return LF_Ok;
}



static void Decode_PID()
{
	if(PID_Module.CalculatedLeftMotorSpeed<0)
		{

			int _CalculatedLeftMotorSpeed=PID_Module.CalculatedLeftMotorSpeed*(-1);

			LeftMotorDrivingReverse(_CalculatedLeftMotorSpeed, PID_Module.CalculatedRightMotorSpeed);
			return;
		}
	if(PID_Module.CalculatedRightMotorSpeed < 0)
		{
		int _CalculatedRightMotorSpeed=PID_Module.CalculatedRightMotorSpeed*(-1);

			RightMotorDrivingReverse(PID_Module.CalculatedLeftMotorSpeed, _CalculatedRightMotorSpeed);

			return;
		}

	ForwardDriving(PID_Module.CalculatedLeftMotorSpeed, PID_Module.CalculatedRightMotorSpeed);
	return;
}



#define EncoderRobot

static void ForwardDriving(int LeftMotorSpeed, int RightMotorSpeed)
{

  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue  - RightMotorSpeed);//-->> Forward
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);

#ifdef EncoderRobot
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue - LeftMotorSpeed );  //-->> Forward
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
#endif

#ifdef SimpleBlackWheel
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue );
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue- LeftMotorSpeed );  //-->> Forward
#endif

}



static void RightMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{

	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue );
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue - RightMotorSpeed);  //-->> Reverse tylu


#ifdef EncoderRobot
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue  - LeftMotorSpeed);  //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
#endif
#ifdef SimpleBlackWheel
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue- LeftMotorSpeed ); //-->> Forward
#endif
}

static void LeftMotorDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue - RightMotorSpeed); //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue );
#ifdef EncoderRobot
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue - LeftMotorSpeed);  //-->> Reverse
#endif
#ifdef SimpleBlackWheel
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue - LeftMotorSpeed); //-->> Reverse
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue );
#endif
}

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

static void EEPROM_ReadTryDetectEndLineMarkState()
{
	int tmp;
	EEPROM_READ_INT(EEPROM_TryDetectEndLineMark_Addr , &tmp);
	Robot_Cntrl.TryDetEndLapMarkState=tmp;
}




