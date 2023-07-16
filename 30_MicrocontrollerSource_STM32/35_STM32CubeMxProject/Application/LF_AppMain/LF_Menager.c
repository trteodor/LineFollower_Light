
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

typedef struct Robot_Cntrl_t
{
	int input_LeftWhPwmVal;
	int input_RightWhPwmVal;

	bool IsFlagStartedForDrivingTime;
	uint32_t RobotStartTime;
	uint32_t RobotRunTime;
}Robot_Cntrl_t;

/*************************************************************************/

Robot_Cntrl_t Robot_Cntrl;

static void MotorsPwmInit();
static void MotorsForceStop();
static void MotorsForwardDriving(int LeftMotorSpeed, int RightMotorSpeed);
static void MotorRightDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void MotorLeftDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);

static void SpeedProfiler();


/**************************************************************************************************/
void MotorsPwmInit()
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
static void MotorsForwardDriving(int LeftMotorSpeed, int RightMotorSpeed)
{
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue  - RightMotorSpeed);//-->> Forward
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);

  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue - LeftMotorSpeed );  //-->> Forward
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
}
/**************************************************************************************************/
static void MotorRightDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{

	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue );
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue - RightMotorSpeed);  //-->> Reverse

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue  - LeftMotorSpeed);  //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
}
/**************************************************************************************************/
static void MotorLeftDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue - RightMotorSpeed); //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue );

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue - LeftMotorSpeed);  //-->> Reverse

}
/**************************************************************************************************/
static void MotorsForceStop(void)
{
	//Motors Off
    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);
    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue);

    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue);
}

static void DecodeLinePid(void)
{
	if(Robot_Cntrl.input_LeftWhPwmVal<0 && (Robot_Cntrl.input_RightWhPwmVal<0) ){
		/*Not handled now*/
	}
	else if(Robot_Cntrl.input_LeftWhPwmVal <= 0){
		int _CalculatedLeftMotorSpeed=Robot_Cntrl.input_LeftWhPwmVal*(-1);
		MotorLeftDrivingReverse(_CalculatedLeftMotorSpeed, Robot_Cntrl.input_RightWhPwmVal);
	}
	else if(Robot_Cntrl.input_RightWhPwmVal < 0){
	    int _CalculatedRightMotorSpeed=Robot_Cntrl.input_RightWhPwmVal*(-1);
		MotorRightDrivingReverse(Robot_Cntrl.input_LeftWhPwmVal, _CalculatedRightMotorSpeed);
	}
	else{ /*Robot_Cntrl.input_LeftWhPwmVal<0 && (Robot_Cntrl.input_RightWhPwmVal<0*/
		MotorsForwardDriving(Robot_Cntrl.input_LeftWhPwmVal, Robot_Cntrl.input_RightWhPwmVal);
	}
}

void LFAppMngrUpdateInputData(void)
{
	App_LinePidGetComputedPwmVals(&Robot_Cntrl.input_LeftWhPwmVal, &Robot_Cntrl.input_RightWhPwmVal);
}

/**************************************************************************************************/
static void SpeedProfiler(void)
{
	static uint8_t speedprofilenumer=0;

	if (speedprofilenumer==1)
	{
		//		if(Enc_Module.TakenDistance > 0)
		//			{
		//			//actions
		//			PID_Module.BaseMotorSpeed=1.0;
		//			}
		//actions
	}
}
/************************************************************************************/
#define API_FUNCTIONS
/************************************************************************************/
void LF_MngrInit(void) /*Line Following Menager init */
{
	// EEPROM_ReadTryDetectEndLineMarkState();
	MotorsPwmInit();
}

void LF_MngrTask(void) /*Line Following Menager task */
{
	LFAppMngrUpdateInputData();
	static uint32_t DrivingStartTime = 0U;
	static bool prevExpectedrDrivingState = false;
	bool ExpectedrDrivingState = BLE_isExpectedStateDriving();

	if(prevExpectedrDrivingState != ExpectedrDrivingState)
	{
		if(true == prevExpectedrDrivingState)
		{/*Changed from driving to standstill*/
			uint32_t DrivingTime = HAL_GetTick() - DrivingStartTime;
			BLE_DbgMsgTransmit("LineFollowing Seconds: %d", (DrivingTime/1000 ));
		}
		else
		{/*Changed from standstill to driving */
			ENC_Init();/*re-init Encoder data*/ 
			DrivingStartTime = HAL_GetTick();
		}
	}

	if(true == ExpectedrDrivingState)
	{
		SpeedProfiler();
		DecodeLinePid();
	}
	else{
		MotorsForceStop();
	}

	prevExpectedrDrivingState = ExpectedrDrivingState;
}

