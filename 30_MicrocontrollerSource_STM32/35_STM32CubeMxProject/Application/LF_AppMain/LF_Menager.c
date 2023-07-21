
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
#include "math.h"

#define LF_M_PI_VAL		((float)3.14159265F)


#define MaxPID_DerivativeTime 200
#define MaxPWMValue 1000 //Depends on the PWM value of the timer counter (Max Timer Count Value dk how to explain :( )


#define CountStatesWhenLineBy4SenDetToEndLapMark 25

/*************************************************************************/

typedef struct Robot_Cntrl_t
{
	int input_LeftWhPwmVal;
	int input_RightWhPwmVal;
	float input_TravelledDistance;
	float input_RobotOrientation;

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

	Robot_Cntrl.input_RobotOrientation = ENC_GetCurrentOrientation();
	Robot_Cntrl.input_TravelledDistance = ENC_GetTravelledDistance();
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

typedef enum{
	rotDirUndefinded,
	rotDirectionLeft,
	rotDirectionRight,
}RotDirection_t;


RotDirection_t RotDirection;


static float  VecLHelperX,VecLHelperY;

void ManualDrivingCallBackHandler(float VecXVal, float VecYVal)
{
VecLHelperX= VecXVal;
VecLHelperY = VecYVal;
 static uint32_t OrientationLoggerTempTimer = 0;
 // input_RobotOrientation
	 if(HAL_GetTick() - OrientationLoggerTempTimer > 150)
	 {
			 	 OrientationLoggerTempTimer = HAL_GetTick();

				float ExpectedOrientation = ( (float)atan2(VecLHelperY,VecLHelperX) );

				if(ExpectedOrientation < (0.0F) )
				{
					ExpectedOrientation =  (LF_M_PI_VAL) + (LF_M_PI_VAL - fabs(ExpectedOrientation));
					/*To get full circle*/
				}

				float CurrOrientationLimited = (Robot_Cntrl.input_RobotOrientation);

				uint32_t OpCounter = 0;
				while( fabs(CurrOrientationLimited) >= ( 2 *LF_M_PI_VAL) )
				{
					if( (CurrOrientationLimited) > (2 *LF_M_PI_VAL) )
					{
						CurrOrientationLimited =  CurrOrientationLimited- (2 *LF_M_PI_VAL) ;
					}
					else{
						CurrOrientationLimited  = CurrOrientationLimited + (2 *LF_M_PI_VAL) ;
					}

					OpCounter++;
				}

				CurrOrientationLimited = fabs(CurrOrientationLimited);


				float NeededRotationRight;
				float NeededRotationLeft;
				float Diff;
				float NeededRot = 0;

				Diff= ExpectedOrientation - CurrOrientationLimited;

				if(Diff > 0)
				{
					NeededRotationLeft = fabs(Diff - 2* LF_M_PI_VAL);
				}else{
					NeededRotationLeft = fabs(Diff);
				}

				if(Diff < 0)
				{
					NeededRotationRight = Diff + ( 2* LF_M_PI_VAL);
				}else{
					NeededRotationRight = fabs(Diff);
				}

				if(NeededRotationLeft > NeededRotationRight)
				{
					NeededRot = -1.0F * NeededRotationRight;
				}
				else{
					NeededRot = NeededRotationLeft;
				}

				BLE_DbgMsgTransmit("NeededRot %f,ExpectedO: %f CurrO: %f " ,NeededRot,ExpectedOrientation,CurrOrientationLimited);
	 }
}

/************************************************************************************/
#define API_FUNCTIONS
/************************************************************************************/
void LF_MngrInit(void) /*Line Following Menager init */
{
	// EEPROM_ReadTryDetectEndLineMarkState();
	BLE_RegisterManualCntrlRequestCallBack(ManualDrivingCallBackHandler);
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
			BLE_DbgMsgTransmit("LineFollowing mSec: %d TakenDist: %f", 
										DrivingTime, Robot_Cntrl.input_TravelledDistance);
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
		// MotorsForceStop();
	}

	prevExpectedrDrivingState = ExpectedrDrivingState;
}

