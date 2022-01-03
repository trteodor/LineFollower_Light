
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdlib.h>
#include "math.h"


#include "EEPROM.h"
#include "EEPROM_VarLocDef.h"
#include "PID_Reg_Module.h"
#include "LineSensorsModule.h"
#include "Encoders_Module.h"

//Global struct
PID_RegModule_t PID_Module;
extern Encoders_Module_t Enc_Module;
/*Few field of the structure above are also modified by HM10Ble App Module*/

static float CalculateSpeedWhichShouldSetForMotors();
static void CheckDoesNotExceedTheMaximumPWMValue();
static float calculatePID();
static void EEPROM_PID_Data_Read();

void PID_Init()
{
	EEPROM_PID_Data_Read();
}

void PID_Task()
{
	SM_GetAndVerifyCalculatedError();
	calculatePID();

	Enc_CalculateActualSpeed();

	CalculateSpeedWhichShouldSetForMotors();
	CheckDoesNotExceedTheMaximumPWMValue();
}

static float calculatePID()
{
		static uint32_t SavedTime_PID_Reg;
		static float PreviousPositionErrorValue;

		PID_Module.P=SensorModule.PositionErrorValue;

		if(PID_Module.PID_DerivativeTime>MaxPID_DerivativeTime){
			PID_Module.PID_DerivativeTime=200; //it don't must exist ;)
		}

		if(SavedTime_PID_Reg + PID_Module.PID_DerivativeTime < HAL_GetTick()){
			PID_Module.D= SensorModule.PositionErrorValue - PreviousPositionErrorValue;
			PreviousPositionErrorValue=SensorModule.PositionErrorValue;
			SavedTime_PID_Reg=HAL_GetTick();

		}
		PID_Module.PID_value = (PID_Module.Kp * PID_Module.P)+(PID_Module.Kd * PID_Module.D);

//		PID_Module.CalculatedLeftMotorSpeed = PID_Module.BaseMotorSpeed + PID_Module.PID_value;
//		PID_Module.CalculatedRightMotorSpeed = PID_Module.BaseMotorSpeed - PID_Module.PID_value;

	return 0;
}

static float CalculateSpeedWhichShouldSetForMotors()
{

	 float PresetSpeed=PID_Module.BaseMotorSpeed;

	 float PresetSpeed_LeftMotor;
	 float PresetSpeed_RightMotor;

	 PresetSpeed_LeftMotor=PresetSpeed+PID_Module.PID_value;
	 PresetSpeed_RightMotor=PresetSpeed-PID_Module.PID_value;

		 if(PresetSpeed_RightMotor >= Enc_Module.RightWheelSpeed)
		 {
			 float delta_sp=PresetSpeed_RightMotor-Enc_Module.RightWheelSpeed;
			 PresetSpeed_RightMotor=PresetSpeed_RightMotor + (delta_sp);
		 }
		 else //(PresetSpeed_RightMotor < Predkosc_P)
		 {
			 float delta_sp=PresetSpeed_RightMotor - Enc_Module.RightWheelSpeed;
			 PresetSpeed_RightMotor=PresetSpeed_RightMotor+(delta_sp);
		 }

		 if(PresetSpeed_LeftMotor > Enc_Module.LeftWheelSpeed)
		 {
			 float delta_sp=PresetSpeed_LeftMotor-Enc_Module.LeftWheelSpeed;
			 PresetSpeed_LeftMotor=PresetSpeed_LeftMotor+(delta_sp);
		 }

		 if(PresetSpeed_LeftMotor < Enc_Module.LeftWheelSpeed)
		 {
			 float delta_pr=PresetSpeed_LeftMotor-Enc_Module.LeftWheelSpeed;
			 Enc_Module.LeftWheelSpeed=Enc_Module.LeftWheelSpeed+(delta_pr);
		 }


		 ///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		 /////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		 ///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		 /////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		 ///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		 /////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		 //Experimentally Designated
		 //Converting PWM value to speed value
//		 RealMotorSpeed = ax +b ex
		PresetSpeed_RightMotor= (A_FactorMotor*PresetSpeed_RightMotor )+B_FactorMotor - (A_FactorMotor* PID_Module.PID_value);
		PresetSpeed_LeftMotor=  (A_FactorMotor*PresetSpeed_LeftMotor  )+B_FactorMotor + (A_FactorMotor* PID_Module.PID_value);

		PID_Module.CalculatedLeftMotorSpeed  = PresetSpeed_LeftMotor;
		PID_Module.CalculatedRightMotorSpeed = PresetSpeed_RightMotor;

return 0;
}

static void CheckDoesNotExceedTheMaximumPWMValue()
{
	if(PID_Module.CalculatedLeftMotorSpeed>MaxPWMValue)
	{
		PID_Module.CalculatedLeftMotorSpeed=MaxPWMValue;
	}
	if(PID_Module.CalculatedLeftMotorSpeed < -MaxPWMValue)
	{
		PID_Module.CalculatedLeftMotorSpeed = -MaxPWMValue;
	}
	if(PID_Module.CalculatedRightMotorSpeed > MaxPWMValue)
	{
		PID_Module.CalculatedRightMotorSpeed = MaxPWMValue;
	}
	if(PID_Module.CalculatedRightMotorSpeed < -MaxPWMValue)
	{
		PID_Module.CalculatedRightMotorSpeed = -MaxPWMValue;
	}
}


static void EEPROM_PID_Data_Read()
{
	EEPROM_READ_INT(EEPROM_PID_KdProbeTime_Addr, &PID_Module.PID_DerivativeTime);
	EEPROM_READ_FLOAT(EEPROM_Addr_PID_Kp, &PID_Module.Kp);
	EEPROM_READ_FLOAT(EEPROM_Addr_PID_Kd, &PID_Module.Kd);
	EEPROM_READ_FLOAT(EEPROM_AddrBaseMotorSpeedValue, &PID_Module.BaseMotorSpeed);
	EEPROM_READ_FLOAT(EEPROM_Addr_PID_Ki, &PID_Module.Ki);
}

