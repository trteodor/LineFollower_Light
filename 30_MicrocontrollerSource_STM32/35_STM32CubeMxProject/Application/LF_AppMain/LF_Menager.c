
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
	float input_LeftWhSpeed;
	float input_RightWhSpeed;

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

/*************************************************************************/
/*Static variables..*/
static Robot_Cntrl_t Robot_Cntrl;
static LinePidReg_t LinePid;

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

	
	EE_ReadVariableF32(EE_NvmAddr_PidKp_F32,&LinePid.Kp);
	EE_ReadVariableF32(EE_NvmAddr_PidKi_F32,&LinePid.Ki);
	EE_ReadVariableF32(EE_NvmAddr_PidKd_F32,&LinePid.Kd);
	EE_ReadVariableU32(EE_NvmAddr_ProbeTime_U32,&LinePid.DerivativeTime);
	EE_ReadVariableF32(EE_NvmAddr_ExpectedMotorSpdValue_F32,&LinePid.BaseMotorSpeed);
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
	Robot_Cntrl.input_LeftWhSpeed = ENC_GetLeftWhSpeed();
	Robot_Cntrl.input_RightWhSpeed = ENC_GetRightWhSpeed();

	LinePid.input_PositionError = LPE_GetPosError();
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
static void ComputeExpectedPwmValues(void)
{
	float ExpectedSpeed=LinePid.BaseMotorSpeed;
	float ExpectedSpeed_LeftMotor;
	float ExpectedSpeed_RightMotor;

	ExpectedSpeed_LeftMotor=ExpectedSpeed + LinePid.PID_value;
	ExpectedSpeed_RightMotor=ExpectedSpeed - LinePid.PID_value;

	float PresetSpeed_LeftMotor=  ExpectedSpeed_LeftMotor;
	float PresetSpeed_RightMotor= ExpectedSpeed_RightMotor;


	if(PresetSpeed_RightMotor >= Robot_Cntrl.input_RightWhSpeed)
	{
		float delta_sp=PresetSpeed_RightMotor-Robot_Cntrl.input_RightWhSpeed;
		PresetSpeed_RightMotor=PresetSpeed_RightMotor + (delta_sp);
	}
	else //(PresetSpeed_RightMotor < Predkosc_P)
	{
		float delta_sp=PresetSpeed_RightMotor - Robot_Cntrl.input_RightWhSpeed;
		PresetSpeed_RightMotor=PresetSpeed_RightMotor+(delta_sp);
	}

	if(PresetSpeed_LeftMotor > Robot_Cntrl.input_LeftWhSpeed)
	{
		float delta_sp=PresetSpeed_LeftMotor-Robot_Cntrl.input_LeftWhSpeed;
		PresetSpeed_LeftMotor=PresetSpeed_LeftMotor+(delta_sp);
	}
	else //PresetSpeed_LeftMotor < Enc_Module.LeftWheelSpeed)
	{
		float delta_pr=PresetSpeed_LeftMotor-Robot_Cntrl.input_LeftWhSpeed;
		PresetSpeed_LeftMotor=PresetSpeed_LeftMotor+(delta_pr);
	}


	//Convert speed value to PWM value 
	//RealMotorSpeed = ax +b
	LinePid.ComputedLeftWhPwmVal= (LinePid.NVM_MotAFacPwmToSpdLeft*PresetSpeed_LeftMotor )+LinePid.NVM_MotBFacPwmToSpdLeft +  (LinePid.NVM_MotAFacPwmToSpdLeft* LinePid.PID_value);
	LinePid.ComputedRightWhPwmVal=  (LinePid.NVM_MotAFacPwmToSpdRight*PresetSpeed_RightMotor)+LinePid.NVM_MotBFacPwmToSpdRight - (LinePid.NVM_MotAFacPwmToSpdRight* LinePid.PID_value);

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
	    int _CalculatedRightMotorSpeed=LinePid.ComputedRightWhPwmVal*(-1);
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
	float PresetSpeed_LeftMotor=  MotSpeedLeft;
	float PresetSpeed_RightMotor= MotSpeedRight;


	if(PresetSpeed_RightMotor >= Robot_Cntrl.input_RightWhSpeed)
	{
		float delta_sp=PresetSpeed_RightMotor-Robot_Cntrl.input_RightWhSpeed;
		PresetSpeed_RightMotor=PresetSpeed_RightMotor + (delta_sp);
	}
	else //(PresetSpeed_RightMotor < Predkosc_P)
	{
		float delta_sp=PresetSpeed_RightMotor - Robot_Cntrl.input_RightWhSpeed;
		PresetSpeed_RightMotor=PresetSpeed_RightMotor+(delta_sp);
	}

	if(PresetSpeed_LeftMotor > Robot_Cntrl.input_LeftWhSpeed)
	{
		float delta_sp=PresetSpeed_LeftMotor-Robot_Cntrl.input_LeftWhSpeed;
		PresetSpeed_LeftMotor=PresetSpeed_LeftMotor+(delta_sp);
	}
	else //PresetSpeed_LeftMotor < Enc_Module.LeftWheelSpeed)
	{
		float delta_pr=PresetSpeed_LeftMotor-Robot_Cntrl.input_LeftWhSpeed;
		PresetSpeed_LeftMotor=PresetSpeed_LeftMotor+(delta_pr);
	}


	int L_ComputedLeftWhPwmVal=   (LinePid.NVM_MotAFacPwmToSpdLeft* PresetSpeed_LeftMotor )+LinePid.NVM_MotBFacPwmToSpdLeft;
	int L_ComputedRightWhPwmVal=  (LinePid.NVM_MotAFacPwmToSpdRight*PresetSpeed_RightMotor )+LinePid.NVM_MotBFacPwmToSpdRight;

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
		/*Not handled now*/
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
	static bool prevExpectedrDrivingState = false;
	bool ExpectedrDrivingState = BLU_isExpectedStateDriving();

	if(false == ExpectedrDrivingState ){
		/*Handle manual driving reqeust by user only if robot state is standstill*/
		if(true == Robot_Cntrl.manualDrivingReqFlag )
		{
			userRequestManualMoveHandler();
		}
	}

	if(prevExpectedrDrivingState != ExpectedrDrivingState)
	{
		if(true == prevExpectedrDrivingState)
		{/*Changed from driving to standstill*/
			uint32_t DrivingTime = HAL_GetTick() - DrivingStartTime;
			BLU_DbgMsgTransmit("LineFollowing mSec: %d TakenDist: %f", 
										DrivingTime, Robot_Cntrl.input_TravelledDistance);

			MotorsForceStop();

		}
		else
		{
			/*Changed from standstill to driving */
			ENC_Init();/*re-init Encoder data*/ 
			DrivingStartTime = HAL_GetTick();
		}
	}

	if(true == ExpectedrDrivingState)
	{
		SpeedProfiler();
		SetMotorSpeedsBaseOnLinePid(); /**/
	}
	else{
		// MotorsForceStop();
	}

	prevExpectedrDrivingState = ExpectedrDrivingState;
}


/************************************************************************************/
#define API_FUNCTIONS
/************************************************************************************/
void LF_MngrInit(void) /*Line Following Menager init */
{
	// EEPROM_ReadTryDetectEndLineMarkState();
	LinePidNvmDataRead();
	BLU_RegisterNvMdataUpdateInfoCallBack(LinePidNvmDataRead);
	BLU_RegisterManualCntrlRequestCallBack(ManualDrivingCallBackRequest);
	MotorsPwmInit();
}

void LF_MngrTask(void) /*Line Following Menager task */
{
	UpdateInputData();
	ComputeLinePidVal();
	ComputeExpectedPwmValues();
	

	ManageRobotMovingState(); /* Totally highest function of robot moving controlling :) */
}




