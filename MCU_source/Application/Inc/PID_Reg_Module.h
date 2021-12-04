#ifndef INC_PID_Reg_Module_H_
#define INC_PID_Reg_Module_H_

#define MaxPID_DerivativeTime 200
#define MaxPWMValue 1000 //Depends on the PWM value of the timer counter (Max Timer Count Value dk how to explain :( )

typedef struct
{
	float Kp;
	float Kd;
	float Ki;
	float PID_value;

	float P;
	float I;
	float D;

	int PID_DerivativeTime;
	float MAX_PID_value;
	float Ki_Sum;
	float Ki_Sum_MaxVal;

	float BaseMotorSpeed;

	int CalculatedLeftMotorSpeed;
	int CalculatedRightMotorSpeed;

	int ReverseSpeed;

}PID_RegModule_t;

extern PID_RegModule_t PID_Module;
/*Few field of the structure above are also modified by HM10Ble App Module*/


void PID_Init();
void PID_Task();

#endif //_PID_Reg_Module_H_
