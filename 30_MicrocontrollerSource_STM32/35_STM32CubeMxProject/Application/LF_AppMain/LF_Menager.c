
#include "main.h"
#include "tim.h"
#include "string.h"
#include <stdio.h>
#include "stdbool.h"
#include "math.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "LF_Menager.h"
#include "EEmu.h"
#include "LedMngr.h"
#include "EncodersHandler.h"
#include "LinePosEstimator.h"
#include "BluetoothClassicComm.h"



#define LF_M_PI_VAL		((float)3.14159265F)
#define MaxPID_DerivativeTime 200
#define MaxPWMValue 1000 //Depends on the PWM value of the timer 
							//counter (Max Timer Count Value dk how to explain :( )
#define CountStatesWhenLineBy4SenDetToEndLapMark 25
#define MaxDerivativeTime 200
#define MaxPWMValue 1000 //Depends on the PWM value of the timer counter (Max Timer Count Value dk how to explain :( )




/*************************************************************************/
typedef struct Robot_Cntrl_t
{
	bool manualDrivingReqFlag;
	float ReqMandrivTimeStamp;
	float input_ReqXvectorVal;
	float input_ReqYvectorVal;

	float input_TravelledDistance;
	float input_RobotOrientation;
	float input_AverageSpeed;
	float input_EncLeftWhSpeed;
	float input_EncRightWhSpeed;

	bool RightRightAngleDetectedFlag;
	bool LeftRightAngleDetectedFlag;

}Robot_Cntrl_t;
/*************************************************************************/
typedef struct LinePidReg_t
{
	float input_PositionError;
	float input_MeasuredLeftWhSpd;
	float input_MeasuredRightWhSpd;

	float PID_value;
	float Kp,Kd,Ki;
	float P,I,D;

	uint32_t DerivativeTime;

	float MAX_PID_value;
	float Ki_Sum;
	float Ki_Sum_MaxVal;

	float BaseMotorSpeed;

	uint32_t NVM_MotAFacPwmToSpdLeft;
	uint32_t NVM_MotAFacPwmToSpdRight;
	uint32_t NVM_MotBFacPwmToSpdLeft;
	uint32_t NVM_MotBFacPwmToSpdRight;

	int ComputedLeftWhPwmVal;
	int ComputedRightWhPwmVal;

	int ReverseSpeed;

}LinePidReg_t;

typedef struct RightAnglePid_Tag{
    uint32_t NVM_RightAgPrTime; //= 20;
	float NVM_RightAgBaseSpdHndlr;
	float NVM_Kp_rAg;
	float NVM_Kd_rAg;
	float NVM_RightAgMaxYawRate;
}RightAnglePid_t;

typedef struct SpeedProfiler_Tag
{
	uint32_t EnabledFlag;
	float TrvDistance[11];
	float BaseSpeedValue[11];
}SpeedProfiler_t;

typedef enum
{
	BTN_UNKNOWN,
	BTN_CHANGE_LF_STATE_REQ,
	BTN_START_LF_PROCESSING,
	BTN_STOP_LF_REQ,

}UsrBtnReq_t;

/*************************************************************************/
/*Static variables..*/
static Robot_Cntrl_t Robot_Cntrl;
static LinePidReg_t LinePid;
static RightAnglePid_t RightAnglePidCfgData;
static SpeedProfiler_t SpeedProfilerData;

static UsrBtnReq_t UsrBtnReqState = BTN_UNKNOWN;

static bool RobotLineFollowingState = false;

static const float NVM_rAgIsBrakeNeedForRightAgThHndlr = 1.5F;
static bool IsBrakingNeededForRightAg = false;

/*************************************************************************/
/*Prototypes*/
static void MotorsPwmInit();
static void MotorsForceStop();
static void MotorsForwardDriving(int LeftMotorSpeed, int RightMotorSpeed);
static void MotorRightDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void MotorLeftDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed);
static void SpeedProfiler();

/*************************************************************************/
static void LinePidNvmDataRead(void)
{
	EE_ReadVariableU32(EE_NvmAddr_MotAtoPwmFacLeft_U32, &LinePid.NVM_MotAFacPwmToSpdLeft);
	EE_ReadVariableU32(EE_NvmAddr_MotAtoPwmFacRight_U32,&LinePid.NVM_MotAFacPwmToSpdRight);
	EE_ReadVariableU32(EE_NvmAddr_MotBtoPwmFacLeft_U32, &LinePid.NVM_MotBFacPwmToSpdLeft);
	EE_ReadVariableU32(EE_NvmAddr_MotBtoPwmFacRight_U32, &LinePid.NVM_MotBFacPwmToSpdRight);

	EE_ReadVariableF32(EE_NvmAddr_LinePidKp_F32,&LinePid.Kp);
	EE_ReadVariableF32(EE_NvmAddr_LinePidKi_F32,&LinePid.Ki);
	EE_ReadVariableF32(EE_NvmAddr_LinePidKd_F32,&LinePid.Kd);
	EE_ReadVariableU32(EE_NvmAddr_ProbeTimeLinePid_U32,&LinePid.DerivativeTime);
	EE_ReadVariableF32(EE_NvmAddr_ExpectedMotorSpdValue_F32,&LinePid.BaseMotorSpeed);

	EE_ReadVariableF32(EE_NvmAddr_RightAgPidKp_F32,&RightAnglePidCfgData.NVM_Kp_rAg);
	EE_ReadVariableF32(EE_NvmAddr_RightAgPidKd_F32,&RightAnglePidCfgData.NVM_Kd_rAg);
	EE_ReadVariableF32(EE_NvmAddr_RightAgBaseSpeed_F32,&RightAnglePidCfgData.NVM_RightAgBaseSpdHndlr);
	EE_ReadVariableF32(EE_NvmAddr_RightAgMaxYawRate_F32,&RightAnglePidCfgData.NVM_RightAgMaxYawRate);
	EE_ReadVariableU32(EE_NvmAddr_PrTimRghtAgPid_U32,&RightAnglePidCfgData.NVM_RightAgPrTime);
}

static void SpeedProfileNvMDataRead(void)
{
		EE_ReadVariableU32(EE_NvmAddr_SpProfileEnableFlag_U32,&SpeedProfilerData.EnabledFlag);

		EE_ReadVariableF32(EE_NvmAddr_SpPofileD01_F32, &SpeedProfilerData.TrvDistance[0]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD02_F32, &SpeedProfilerData.TrvDistance[1]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD03_F32, &SpeedProfilerData.TrvDistance[2]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD04_F32, &SpeedProfilerData.TrvDistance[3]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD05_F32, &SpeedProfilerData.TrvDistance[4]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD06_F32, &SpeedProfilerData.TrvDistance[5]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD07_F32, &SpeedProfilerData.TrvDistance[6]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD08_F32, &SpeedProfilerData.TrvDistance[7]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD09_F32, &SpeedProfilerData.TrvDistance[8]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD10_F32, &SpeedProfilerData.TrvDistance[9]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileD11_F32, &SpeedProfilerData.TrvDistance[10]);

		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp01_F32, &SpeedProfilerData.BaseSpeedValue[0]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp02_F32, &SpeedProfilerData.BaseSpeedValue[1]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp03_F32, &SpeedProfilerData.BaseSpeedValue[2]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp04_F32, &SpeedProfilerData.BaseSpeedValue[3]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp05_F32, &SpeedProfilerData.BaseSpeedValue[4]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp06_F32, &SpeedProfilerData.BaseSpeedValue[5]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp07_F32, &SpeedProfilerData.BaseSpeedValue[6]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp08_F32, &SpeedProfilerData.BaseSpeedValue[7]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp09_F32, &SpeedProfilerData.BaseSpeedValue[8]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp10_F32, &SpeedProfilerData.BaseSpeedValue[9]);
		EE_ReadVariableF32(EE_NvmAddr_SpPofileBase_Sp11_F32, &SpeedProfilerData.BaseSpeedValue[10]);
}

/**************************************************************************************************/
static void MotorsPwmInit()
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
static void UpdateInputData(void)
{
	Robot_Cntrl.input_RobotOrientation = ENC_GetCurrentOrientation();
	Robot_Cntrl.input_TravelledDistance = ENC_GetTravelledDistance();
	Robot_Cntrl.input_AverageSpeed = ENC_GetAverageSpeed();
	Robot_Cntrl.input_EncLeftWhSpeed = ENC_GetLeftWhSpeed();
	Robot_Cntrl.input_EncRightWhSpeed = ENC_GetRightWhSpeed();

	LinePid.input_PositionError = LPE_GetPosError();
}
/**************************************************************************************************/
static void MotorsForwardDriving(int LeftMotorSpeed, int RightMotorSpeed)
{
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue);//-->> Forward
  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue - RightMotorSpeed);

  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue );  //-->> Forward
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue - LeftMotorSpeed);
}
/**************************************************************************************************/
static void MotorRightDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{

	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue - RightMotorSpeed );
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);  //-->> Reverse

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue);  //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue - LeftMotorSpeed);
}
/**************************************************************************************************/
static void MotorLeftDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue); //-->> Forward
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue - RightMotorSpeed );

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue - LeftMotorSpeed);
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue );  //-->> Reverse

}
static void MotorBothDrivingReverse(int LeftMotorSpeed, int RightMotorSpeed)
{
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,MaxPWMValue - RightMotorSpeed );
	  __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,MaxPWMValue);  //-->> Reverse

	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MaxPWMValue - LeftMotorSpeed);
	  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,MaxPWMValue );  //-->> Reverse

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
/**************************************************************************************************/
static void ComputeLinePidVal(void)
{
		static uint32_t SavedTime_PID_Reg;
		static float PreviousPositionErrorValue;
		
		LinePid.P = LinePid.input_PositionError;

		if( (HAL_GetTick() - SavedTime_PID_Reg) > LinePid.DerivativeTime ){
			LinePid.D= LinePid.input_PositionError - PreviousPositionErrorValue;
			PreviousPositionErrorValue=LinePid.input_PositionError;
			SavedTime_PID_Reg=HAL_GetTick();
		}

		LinePid.PID_value = (LinePid.Kp * LinePid.P)+(LinePid.Kd * LinePid.D);
}
/**************************************************************************************************/
static void CompensateBatteryDischarge(float *ExpectedSpeedLftMot,float *ExpectedSpdRightMot)
{
	static const float DeltaKpIfSpeedIsTooLow = 2;

	if(*ExpectedSpdRightMot >= Robot_Cntrl.input_EncRightWhSpeed)
	{
		float delta_sp=fabs(*ExpectedSpdRightMot-Robot_Cntrl.input_EncRightWhSpeed);
		*ExpectedSpdRightMot=*ExpectedSpdRightMot + (delta_sp * DeltaKpIfSpeedIsTooLow);
	}
	else //(*ExpectedSpdRightMot < Robot_Cntrl.input_EncRightWhSpeed)
	{
		float delta_sp=fabs(*ExpectedSpdRightMot - Robot_Cntrl.input_EncRightWhSpeed);
		*ExpectedSpdRightMot=*ExpectedSpdRightMot - (delta_sp);
	}

	if(*ExpectedSpeedLftMot > Robot_Cntrl.input_EncLeftWhSpeed)
	{
		float delta_sp=fabs(*ExpectedSpeedLftMot-Robot_Cntrl.input_EncLeftWhSpeed);
		*ExpectedSpeedLftMot=*ExpectedSpeedLftMot + (delta_sp * DeltaKpIfSpeedIsTooLow);
	}
	else //(*ExpectedSpeedLftMot < Robot_Cntrl.input_EncLeftWhSpeed)
	{
		float delta_pr=fabs(*ExpectedSpeedLftMot-Robot_Cntrl.input_EncLeftWhSpeed);
		*ExpectedSpeedLftMot=*ExpectedSpeedLftMot - (delta_pr);
	}
}

static void ComputeExpectedPwmValues(void)
{
	float ExpectedSpeed=LinePid.BaseMotorSpeed;
	float ExpectedSpeed_LeftMotor;
	float ExpectedSpeed_RightMotor;

	ExpectedSpeed_LeftMotor=ExpectedSpeed + LinePid.PID_value;
	ExpectedSpeed_RightMotor=ExpectedSpeed - LinePid.PID_value;

	float ExpectedPresetSpeed_LeftMotor=  ExpectedSpeed_LeftMotor;
	float ExpectedPresetSpeed_RightMotor= ExpectedSpeed_RightMotor;

	CompensateBatteryDischarge(&ExpectedPresetSpeed_LeftMotor,&ExpectedPresetSpeed_RightMotor);


	//Convert speed value to PWM value 
	//RealMotorSpeed = ax +b
	LinePid.ComputedLeftWhPwmVal= (LinePid.NVM_MotAFacPwmToSpdLeft*ExpectedPresetSpeed_LeftMotor )+LinePid.NVM_MotBFacPwmToSpdLeft +  (LinePid.NVM_MotAFacPwmToSpdLeft* LinePid.PID_value);
	LinePid.ComputedRightWhPwmVal=  (LinePid.NVM_MotAFacPwmToSpdRight*ExpectedPresetSpeed_RightMotor)+LinePid.NVM_MotBFacPwmToSpdRight - (LinePid.NVM_MotAFacPwmToSpdRight* LinePid.PID_value);

	if(LinePid.ComputedLeftWhPwmVal>MaxPWMValue){
		LinePid.ComputedLeftWhPwmVal=MaxPWMValue;
	}
	else if(LinePid.ComputedLeftWhPwmVal < -MaxPWMValue){
		LinePid.ComputedLeftWhPwmVal = -MaxPWMValue;
	}
	
	if(LinePid.ComputedRightWhPwmVal > MaxPWMValue){
		LinePid.ComputedRightWhPwmVal = MaxPWMValue;
	}
	else if(LinePid.ComputedRightWhPwmVal < -MaxPWMValue){
		LinePid.ComputedRightWhPwmVal = -MaxPWMValue;
	}
}
/**************************************************************************************************/
static void SetMotorSpeedsBaseOnLinePid(void)
{
	if(LinePid.ComputedLeftWhPwmVal<0 && (LinePid.ComputedRightWhPwmVal<0) ){
		/*Not handled now*/
	}
	else if(LinePid.ComputedLeftWhPwmVal <= 0){
		int _CalculatedLeftMotorSpeed=LinePid.ComputedLeftWhPwmVal*(-1);
		MotorLeftDrivingReverse(_CalculatedLeftMotorSpeed, LinePid.ComputedRightWhPwmVal);
	}
	else if(LinePid.ComputedRightWhPwmVal < 0){
	    int _CalculatedRightMotorSpeed=LinePid.ComputedRightWhPwmVal*(-1); /*Revert*/
		MotorRightDrivingReverse(LinePid.ComputedLeftWhPwmVal, _CalculatedRightMotorSpeed);
	}
	else{ /*LinePid.ComputedLeftWhPwmVal>0 && (LinePid.ComputedRightWhPwmVal>0*/
		MotorsForwardDriving(LinePid.ComputedLeftWhPwmVal, LinePid.ComputedRightWhPwmVal);
	}
}
/**************************************************************************************************/

/*!
 ************************************************************************************************
 * \brief SetMotorSpeed(float MotSpeedLeft, float MotSpeedRight)
 * \details -- This function sets the Pwm values of the motor based on the input of expected speed in m/s
 * \param in MotSpeedLeft - left motor speed passed as expected m/s
 * \param in MotSpeedRight - right motor speed passed as expected m/s
 * */
static void SetMotorSpeeds(float MotSpeedLeft, float MotSpeedRight)
{
	float ExpectedPresetSpeed_LeftMotor=  MotSpeedLeft;
	float ExpectedPresetSpeed_RightMotor= MotSpeedRight;

	CompensateBatteryDischarge(&ExpectedPresetSpeed_LeftMotor,&ExpectedPresetSpeed_RightMotor);

	int L_ComputedLeftWhPwmVal=   (LinePid.NVM_MotAFacPwmToSpdLeft* ExpectedPresetSpeed_LeftMotor )+LinePid.NVM_MotBFacPwmToSpdLeft;
	int L_ComputedRightWhPwmVal=  (LinePid.NVM_MotAFacPwmToSpdRight*ExpectedPresetSpeed_RightMotor )+LinePid.NVM_MotBFacPwmToSpdRight;

	if(L_ComputedLeftWhPwmVal>MaxPWMValue){
		L_ComputedLeftWhPwmVal=MaxPWMValue;
	}
	else if(L_ComputedLeftWhPwmVal < -MaxPWMValue){
		L_ComputedLeftWhPwmVal = -MaxPWMValue;
	}
	
	if(L_ComputedRightWhPwmVal > MaxPWMValue){
		L_ComputedRightWhPwmVal = MaxPWMValue;
	}
	else if(L_ComputedRightWhPwmVal < -MaxPWMValue){
		L_ComputedRightWhPwmVal = -MaxPWMValue;
	}


	if(L_ComputedLeftWhPwmVal<0 && (L_ComputedRightWhPwmVal<0) ){
		MotorBothDrivingReverse(L_ComputedLeftWhPwmVal,L_ComputedRightWhPwmVal);
	}
	else if(L_ComputedLeftWhPwmVal <= 0){
		int _CalculatedLeftMotorSpeed=L_ComputedLeftWhPwmVal*(-1);
		MotorLeftDrivingReverse(_CalculatedLeftMotorSpeed, L_ComputedRightWhPwmVal);
	}
	else if(L_ComputedRightWhPwmVal < 0){
	    int _CalculatedRightMotorSpeed=L_ComputedRightWhPwmVal*(-1);
		MotorRightDrivingReverse(L_ComputedLeftWhPwmVal, _CalculatedRightMotorSpeed);
	}
	else{ /*L_ComputedLeftWhPwmVal>0 && (L_ComputedRightWhPwmVal>0*/
		MotorsForwardDriving(L_ComputedLeftWhPwmVal, L_ComputedRightWhPwmVal);
	}
}

static void SpeedProfiler(void)
{

	if (true == SpeedProfilerData.EnabledFlag)
	{
		if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[10])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[10];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[9])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[9];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[8])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[8];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[7])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[7];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[6])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[6];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[5])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[5];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[4])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[4];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[3])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[3];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[2])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[2];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[1])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[1];
		}
		else if(ENC_GetTravelledDistance() > SpeedProfilerData.TrvDistance[0])
		{
			LinePid.BaseMotorSpeed = SpeedProfilerData.BaseSpeedValue[0];
		}
	}
}
/**************************************************************************************************/

/*!
 ************************************************************************************************
 * \brief estimateNeededRotation
 * \details --
 * \param RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param in VecXVal
 * \param in VecXVal
 * \return Returned positive value is meaning that vehicle should rotate to right to achive expected orientation
 * */
float estimateNeededRotation(float VecXVal, float VecYVal)
{
				float ExpectedOrientation = ( (float)atan2(VecYVal,VecXVal) );

				if(ExpectedOrientation < (0.0F) )
				{
					ExpectedOrientation =  ( (LF_M_PI_VAL) + (LF_M_PI_VAL - fabs(ExpectedOrientation)) );
					/*To get full circle*/
				}

				ExpectedOrientation = ( 2 *LF_M_PI_VAL) - ExpectedOrientation; /*invert*/

				float CurrOrientationLimited = (Robot_Cntrl.input_RobotOrientation);

				while( fabs(CurrOrientationLimited) >= ( 2 *LF_M_PI_VAL) )
				{
					if( (CurrOrientationLimited) > (2 *LF_M_PI_VAL) )
					{
						CurrOrientationLimited =  CurrOrientationLimited- (2 *LF_M_PI_VAL) ;
					}
					else{
						CurrOrientationLimited  = CurrOrientationLimited + (2 *LF_M_PI_VAL) ;
					}
				}
				if(CurrOrientationLimited < 0.0F)
				{
					CurrOrientationLimited = CurrOrientationLimited + (2 *LF_M_PI_VAL);
				}

				float NeededRotationRight;
				float NeededRotationLeft;
				float Diff;
				float NeededRot = 0;

				Diff= ExpectedOrientation - CurrOrientationLimited;

				if(Diff > 0){
					NeededRotationLeft = fabs(Diff - 2* LF_M_PI_VAL);
				}else{
					NeededRotationLeft = fabs(Diff);
				}

				if(Diff < 0){
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


		// static uint32_t OrientationLoggerTempTimer = 0;
		// if(HAL_GetTick() - OrientationLoggerTempTimer > 150)
		// {	
		// 	OrientationLoggerTempTimer = HAL_GetTick();
		// 	BLU_DbgMsgTransmit("NeededRot:%f Cuur:%f ExpOri:%f " ,NeededRot,CurrOrientationLimited,ExpectedOrientation);
		// }

	return NeededRot;
}
/**************************************************************************************************/
void ManualDrivingCallBackRequest(float VecXVal, float VecYVal)
{
	Robot_Cntrl.input_ReqYvectorVal = VecYVal;
	Robot_Cntrl.input_ReqXvectorVal = VecXVal;
	Robot_Cntrl.manualDrivingReqFlag = true;
	Robot_Cntrl.ReqMandrivTimeStamp = HAL_GetTick();
}

void LfMngr_LineEventCallBack(LinePosEstimatorEvent_t LinePosEstEv)
{
	if(LinePosEstEv == Event_RightRightAngle)
	{
		BLU_DbgMsgTransmit("Right|| RightAngleDetected TrvD: %.3f",ENC_GetTravelledDistance() );
		Robot_Cntrl.RightRightAngleDetectedFlag = true;
	}
	if(LinePosEstEv == Event_LeftRightAngle)
	{
		BLU_DbgMsgTransmit("Left || RightAngleDetected TrvD: %.3f",ENC_GetTravelledDistance() );
		Robot_Cntrl.LeftRightAngleDetectedFlag = true;
	}

}

static void MonitorVehSpdToHandleRightAg(void)
{
	static float vehSpdBuffer[10];
	static uint8_t vehSpdBufferIterator = 0;
	static uint32_t monitorVehSpdTimer = 0;
	float currVehSpd = ENC_GetVehicleSpeed();

	if(currVehSpd > NVM_rAgIsBrakeNeedForRightAgThHndlr)
	{
		float averageVehSpeed = 0.0F;

		if(vehSpdBufferIterator > 10){
			vehSpdBufferIterator = 0;
		}
		if(HAL_GetTick() - monitorVehSpdTimer > 10)
		{
			vehSpdBuffer[vehSpdBufferIterator] = currVehSpd;
		}
		vehSpdBufferIterator++;

		for(int i=0; i<10; i++)
		{
			averageVehSpeed += vehSpdBuffer[i] / 10.0F;
		}
		if(averageVehSpeed > NVM_rAgIsBrakeNeedForRightAgThHndlr)
		{
			IsBrakingNeededForRightAg = true;
		}
	}
	else
	{
		for(int i=0; i<10; i++){
			vehSpdBuffer[i] = 0.0F;
		}

		IsBrakingNeededForRightAg = false;
	}


}

void HandleRightAngle(void)
{
	static bool RightAngleHandlingStartedFlag = false;
	static float expectedOrientation = 0U;

	static bool brakingFlag = false;
	static uint32_t brakingTimer= 0U;

	static bool PrevIsBrakingNeededForRightAg = false;

	if(false == RightAngleHandlingStartedFlag)
	{
		if(true == Robot_Cntrl.RightRightAngleDetectedFlag)
		{
			expectedOrientation =  ENC_GetCurrentOrientation() + (LF_M_PI_VAL/ 1.45); //+90deg
			BLU_DbgMsgTransmit("HandleRightAngle:  RR_NewExpOr: %f CurrO: %f", expectedOrientation,ENC_GetCurrentOrientation() );
		}
		else if(true == Robot_Cntrl.LeftRightAngleDetectedFlag)
		{
			expectedOrientation = ENC_GetCurrentOrientation() - (LF_M_PI_VAL/ 1.45F); //-90deg
			BLU_DbgMsgTransmit("HandleRightAngle:  RL_NewExpOr: %f CurrO: %f", expectedOrientation,ENC_GetCurrentOrientation() );

		}
		if(IsBrakingNeededForRightAg)
		{
			PrevIsBrakingNeededForRightAg = IsBrakingNeededForRightAg;
			brakingFlag = true;
			brakingTimer = HAL_GetTick();
		}
		RightAngleHandlingStartedFlag = true;
	}

	if( (true == brakingFlag) &&
		(true == RightAngleHandlingStartedFlag) &&
		(true == PrevIsBrakingNeededForRightAg) )
	{
		if(HAL_GetTick() - brakingTimer > 80)
		{
			brakingFlag = false;
			PrevIsBrakingNeededForRightAg = false;
		}else
		{
			if(true == Robot_Cntrl.RightRightAngleDetectedFlag)
			{
				SetMotorSpeeds(-1.5F,-5.0F);
			}
			else if(true == Robot_Cntrl.LeftRightAngleDetectedFlag)
			{
				SetMotorSpeeds(-5.0F,-1.5F);
			}
		}
	}

	if(true == RightAngleHandlingStartedFlag && false == brakingFlag) //
	{
		static uint32_t SavedTimeR_AgDriv_PID=0;
		static float PreviousPositionErrorValueR_AgDrivPid=0;
		static float R_AgDrivPid_P,R_AgDrivPid_D;
		static float PidR_AgDrivResult;

		R_AgDrivPid_P = ( ENC_GetCurrentOrientation() - expectedOrientation ); /*Tract needed rotation as error*/

		if( (HAL_GetTick() - SavedTimeR_AgDriv_PID) > RightAnglePidCfgData.NVM_RightAgPrTime ){
			R_AgDrivPid_D= R_AgDrivPid_P - PreviousPositionErrorValueR_AgDrivPid;
			PreviousPositionErrorValueR_AgDrivPid=R_AgDrivPid_P;
			SavedTimeR_AgDriv_PID=HAL_GetTick();
		}
		PidR_AgDrivResult = (RightAnglePidCfgData.NVM_Kp_rAg * R_AgDrivPid_P)+(RightAnglePidCfgData.NVM_Kd_rAg * R_AgDrivPid_D);

		SetMotorSpeeds(RightAnglePidCfgData.NVM_RightAgBaseSpdHndlr-PidR_AgDrivResult,
							RightAnglePidCfgData.NVM_RightAgBaseSpdHndlr+PidR_AgDrivResult);

		static uint32_t RightAngleOrientationLoggingTimer= 0;

		if(HAL_GetTick() - RightAngleOrientationLoggingTimer > 100)
		{
			RightAngleOrientationLoggingTimer = HAL_GetTick();
			BLU_DbgMsgTransmit("RightAnglePrLogging:  RL_NewExpOr: %f CurrO: %f", expectedOrientation,ENC_GetCurrentOrientation() );
		}

		if( (fabs(ENC_GetCurrentOrientation() - expectedOrientation) < 0.1F) || (LPE_GetPosError() == 0))
		{
			Robot_Cntrl.RightRightAngleDetectedFlag = false;
			Robot_Cntrl.LeftRightAngleDetectedFlag = false;
			RightAngleHandlingStartedFlag = false;
		}
	}
}

void userRequestManualMoveHandler(void)
{
	static uint32_t SavedTimeManDriv_PID;
	static float PreviousPositionErrorValueManDrivPid;
	static float ManDrivPid_P,ManDrivPid_D;
	static float PidManDrivResult;
	static const uint32_t ProbeTimeManDrivPid = 20;
	static const float ManDrivPid_Kp = 0.45F;
	static const float ManDrivPid_Kd = 0.3F;
	static const float BaseSpdManMove = 0.5F;

	if(Robot_Cntrl.input_ReqYvectorVal != 0.0F && Robot_Cntrl.input_ReqXvectorVal != 0.0F)
	{
		float neededRot = estimateNeededRotation(Robot_Cntrl.input_ReqXvectorVal, Robot_Cntrl.input_ReqYvectorVal);


		ManDrivPid_P = neededRot; /*Tract needed rotation as error*/
		if(ManDrivPid_P > 2.0F)
		{
			ManDrivPid_P = 2.0F;
		}

		if(ManDrivPid_P < (-2.0F) )
		{
			ManDrivPid_P = (-2.0F);
		}

		if( (HAL_GetTick() - SavedTimeManDriv_PID) > ProbeTimeManDrivPid ){
			ManDrivPid_D= ManDrivPid_P - PreviousPositionErrorValueManDrivPid;
			PreviousPositionErrorValueManDrivPid=ManDrivPid_P;
			SavedTimeManDriv_PID=HAL_GetTick();
		}
		PidManDrivResult = (ManDrivPid_Kp * ManDrivPid_P)+(ManDrivPid_Kd * ManDrivPid_D);

		SetMotorSpeeds(BaseSpdManMove-PidManDrivResult,BaseSpdManMove+PidManDrivResult);

	}
	else
	{
		Robot_Cntrl.manualDrivingReqFlag = false;
		MotorsForceStop();
	} 
}

/**************************************************************************************************/
void ManageRobotMovingState(void)
{
	static uint32_t DrivingStartTime = 0U;
	static bool prevExpectedrDrivingStateBluetooth = false;
	bool ExpectedrDrivingStateBluetooth = BLU_isExpectedStateDriving();
	static uint32_t usrBtnDelayTimer;
	static bool usrBtnForcedFollowState = false;
	static bool usrBtnForcedFollowState_prev = false;



	if(UsrBtnReqState == BTN_CHANGE_LF_STATE_REQ)
	{
		if(true == usrBtnForcedFollowState)
		{
			usrBtnForcedFollowState = false;
		}
		else{
			usrBtnDelayTimer = HAL_GetTick();
			UsrBtnReqState = BTN_START_LF_PROCESSING;
		}
	}
	else if(UsrBtnReqState == BTN_START_LF_PROCESSING 
	   && HAL_GetTick() - usrBtnDelayTimer > 3000)
	{
		usrBtnForcedFollowState = true;
		UsrBtnReqState = BTN_UNKNOWN;
	}
	else if(UsrBtnReqState == BTN_STOP_LF_REQ )
	{
		usrBtnForcedFollowState = false;
		UsrBtnReqState = BTN_UNKNOWN;
	}

	if(true == ExpectedrDrivingStateBluetooth || true == usrBtnForcedFollowState)
	{
		RobotLineFollowingState = true;
	}else{
		RobotLineFollowingState = false;
	}

	if(false == ExpectedrDrivingStateBluetooth && false == usrBtnForcedFollowState){
		/*Handle manual driving reqeust by user only if robot state is standstill*/
		if(true == Robot_Cntrl.manualDrivingReqFlag )
		{
			userRequestManualMoveHandler();
		}
	}

	if( (prevExpectedrDrivingStateBluetooth != ExpectedrDrivingStateBluetooth) 
	    || usrBtnForcedFollowState != usrBtnForcedFollowState_prev)
	{

		if(true == prevExpectedrDrivingStateBluetooth || true == usrBtnForcedFollowState_prev)
		{/*Changed from driving to standstill*/
			BLU_DbgMsgTransmit("LineFollower stop");
			uint32_t DrivingTime = HAL_GetTick() - DrivingStartTime;
			BLU_DbgMsgTransmit("Following mS: %d takeDist: %.3f AvSpd: %.3f ", 
										DrivingTime, Robot_Cntrl.input_TravelledDistance,Robot_Cntrl.input_AverageSpeed);

			MotorsForceStop();

			if(usrBtnForcedFollowState == true){
				usrBtnForcedFollowState = false;
			}
		}
		else
		{
			BLU_DbgMsgTransmit("LineFollower start!");
			/*Changed from standstill to driving */
			ENC_Init();/*re-init Encoder data*/ 
			DrivingStartTime = HAL_GetTick();
		}
	}

	if(true == ExpectedrDrivingStateBluetooth || true == usrBtnForcedFollowState)
	{

		if(true == Robot_Cntrl.RightRightAngleDetectedFlag || true == Robot_Cntrl.LeftRightAngleDetectedFlag)
		{
			HandleRightAngle();
		}
		else
		{
			SpeedProfiler();
			SetMotorSpeedsBaseOnLinePid(); /**/
		}
	}
	else{
		// MotorsForceStop();
	}

	prevExpectedrDrivingStateBluetooth = ExpectedrDrivingStateBluetooth;
	usrBtnForcedFollowState_prev = usrBtnForcedFollowState;
}


/************************************************************************************/
#define API_FUNCTIONS
/************************************************************************************/
void LF_MngrInit(void) /*Line Following Menager init */
{
	LinePidNvmDataRead();
	SpeedProfileNvMDataRead();
	BLU_RegisterNvMdataUpdateInfoCallBack(LinePidNvmDataRead);
	BLU_RegisterNvMdataUpdateInfoCallBack(SpeedProfileNvMDataRead);
	BLU_RegisterManualCntrlRequestCallBack(ManualDrivingCallBackRequest);
	LPE_RegisterLineEventCallBack(LfMngr_LineEventCallBack);
	MotorsPwmInit();
}

void CheckUserButtonState(void)
{
	static uint32_t buttonStateDebounceTimer = 0;
	static bool alreadyClickHandledFlag = false;
										/*Reset mean button pushed..*/
	if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin) == GPIO_PIN_RESET )
	{
		if(HAL_GetTick() - buttonStateDebounceTimer > 1500 && alreadyClickHandledFlag == false)
		{
			LED_DoQuickBlinks(30);
			alreadyClickHandledFlag = true;
			UsrBtnReqState = BTN_CHANGE_LF_STATE_REQ;
		}
		else if(RobotLineFollowingState == true)
		{
			UsrBtnReqState = BTN_STOP_LF_REQ;
			alreadyClickHandledFlag = true;
		}
	}
	else{
		buttonStateDebounceTimer = HAL_GetTick();
		alreadyClickHandledFlag = false;
	}
	
}

void ReportPidRegValue(void)
{
	BLU_PidRegData_t LinePidRegData;
	LinePidRegData.LinePidRegVal = LinePid.PID_value;

	BLU_ReportLinePid(&LinePidRegData);
}


void LF_MngrTask(void) /*Line Following Menager task */
{
	UpdateInputData();
	ComputeLinePidVal();
	ComputeExpectedPwmValues();
	CheckUserButtonState();
	MonitorVehSpdToHandleRightAg();

	ManageRobotMovingState(); /* Totally highest function of robot moving controlling :) */

	ReportPidRegValue();
}




