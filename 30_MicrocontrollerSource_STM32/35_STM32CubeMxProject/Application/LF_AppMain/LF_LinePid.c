#include "stdint.h"

#include "LF_AppConfig.h"
#include "LF_LinePid.h"
#include "LinePosEstimator.h"
#include "EEmu.h"
#include "BluetoothClassicComm.h"

#define MaxDerivativeTime 200
#define MaxPWMValue 1000 //Depends on the PWM value of the timer counter (Max Timer Count Value dk how to explain :( )

typedef struct LinePidReg_t
{
	float input_PositionError;
	float input_MeasuredLeftWhSpd;
	float input_MeasuredRightWhSpd;

	float PID_value;

	float Kp;
	float Kd;
	float Ki;
	float P;
	float I;
	float D;

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

LinePidReg_t LinePid;


static void LinePidNvmDataRead(void)
{
	EE_ReadVariableU32(EE_NvmAddr_MotAtoPwmFacLeft_U32, &LinePid.NVM_MotAFacPwmToSpdLeft);
	EE_ReadVariableU32(EE_NvmAddr_MotAtoPwmFacRight_U32,&LinePid.NVM_MotAFacPwmToSpdRight);
	EE_ReadVariableU32(EE_NvmAddr_MotBtoPwmFacLeft_U32, &LinePid.NVM_MotBFacPwmToSpdLeft);
	EE_ReadVariableU32(EE_NvmAddr_MotBtoPwmFacRight_U32, &LinePid.NVM_MotBFacPwmToSpdRight);

	
	EE_ReadVariableF32(EE_NvmAddr_PidKp_F32,&LinePid.Kp);
	EE_ReadVariableF32(EE_NvmAddr_PidKi_F32,&LinePid.Ki);
	EE_ReadVariableF32(EE_NvmAddr_PidKd_F32,&LinePid.Kd);
	EE_ReadVariableU32(EE_NvmAddr_ProbeTime_U32,&LinePid.DerivativeTime);
	EE_ReadVariableF32(EE_NvmAddr_ExpectedMotorSpdValue_F32,&LinePid.BaseMotorSpeed);
}

static void ComputeLinePidVal(void)
{
		static uint32_t SavedTime_PID_Reg;
		static float PreviousPositionErrorValue;
		
		LinePid.P = LinePid.input_PositionError;

		if(LinePid.DerivativeTime>MaxDerivativeTime){
			LinePid.DerivativeTime=200; //it don't must exist ;)
		}

		if( (HAL_GetTick() - SavedTime_PID_Reg) > LinePid.DerivativeTime ){
			LinePid.D= LinePid.input_PositionError - PreviousPositionErrorValue;
			PreviousPositionErrorValue=LinePid.input_PositionError;
			SavedTime_PID_Reg=HAL_GetTick();
		}
		LinePid.PID_value = (LinePid.Kp * LinePid.P)+(LinePid.Kd * LinePid.D);
}

static void ComputeExpectedPwmValues(void)
{

	float ExpectedSpeed=LinePid.BaseMotorSpeed;

	float ExpectedSpeed_LeftMotor;
	float ExpectedSpeed_RightMotor;

	ExpectedSpeed_LeftMotor=ExpectedSpeed + LinePid.PID_value;
	ExpectedSpeed_RightMotor=ExpectedSpeed - LinePid.PID_value;

	//Convert speed value to PWM value 
	//RealMotorSpeed = ax +b ex
	LinePid.ComputedLeftWhPwmVal= (LinePid.NVM_MotAFacPwmToSpdLeft*ExpectedSpeed_LeftMotor )+LinePid.NVM_MotBFacPwmToSpdLeft +  (LinePid.NVM_MotAFacPwmToSpdLeft* LinePid.PID_value);
	LinePid.ComputedRightWhPwmVal=  (LinePid.NVM_MotAFacPwmToSpdRight*ExpectedSpeed_RightMotor)+LinePid.NVM_MotBFacPwmToSpdRight - (LinePid.NVM_MotAFacPwmToSpdRight* LinePid.PID_value);

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


void UpdateInputData(void)
{
	LinePid.input_PositionError = LPE_GetPosError();
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
#define API_FUNC_IMP_LINE_PID

void App_LinePidInit(void)
{
	LinePidNvmDataRead();
	BLU_RegisterNvMdataUpdateInfoCallBack(LinePidNvmDataRead);
}

void App_LinePidTask(void)
{
	UpdateInputData();
	ComputeLinePidVal();
	ComputeExpectedPwmValues();
}

void App_LinePidGetComputedPwmVals(int *LeftWhExpectedPwm, int *RightWhExpectedPwm)
{
	*LeftWhExpectedPwm = LinePid.ComputedLeftWhPwmVal;
	*RightWhExpectedPwm = LinePid.ComputedRightWhPwmVal;
}

void App_LinePidComputeExpectedPwmValuesForExpSpd(float ExpectedSpeed,int *LeftWhExpectedPwm, int *RightWhExpectedPwm)
{
	float ExpectedSpeed_LeftMotor;
	float ExpectedSpeed_RightMotor;

	ExpectedSpeed_LeftMotor=ExpectedSpeed + LinePid.PID_value;
	ExpectedSpeed_RightMotor=ExpectedSpeed - LinePid.PID_value;

	//Convert speed value to PWM value 
	//RealMotorSpeed = ax +b ex
	LinePid.ComputedLeftWhPwmVal= (LinePid.NVM_MotAFacPwmToSpdLeft*ExpectedSpeed_LeftMotor )+LinePid.NVM_MotBFacPwmToSpdLeft +  (LinePid.NVM_MotAFacPwmToSpdLeft* LinePid.PID_value);
	LinePid.ComputedRightWhPwmVal=  (LinePid.NVM_MotAFacPwmToSpdRight*ExpectedSpeed_RightMotor)+LinePid.NVM_MotBFacPwmToSpdRight - (LinePid.NVM_MotAFacPwmToSpdRight* LinePid.PID_value);

	if(LinePid.ComputedLeftWhPwmVal>MaxPWMValue){
		LinePid.ComputedRightWhPwmVal=MaxPWMValue;
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

	*LeftWhExpectedPwm = LinePid.ComputedLeftWhPwmVal;
	*RightWhExpectedPwm = LinePid.ComputedRightWhPwmVal;
}
