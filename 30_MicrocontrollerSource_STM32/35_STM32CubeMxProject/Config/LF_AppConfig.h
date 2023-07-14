#ifndef _INC_LF_AppConfig_H
#define _INC_LF_AppConfig_H

//EncoderDecoder
//This file is created to avoid any magic numbers...
#define OneImpulsDistance 0.000788 //depends on the size of the wheel and the gear on the motor
#define MaxProbeNumber 4000 //Only My Decision 20ms*4000=80seconds - inaf

#define EncodersProbeTime 200 //20ms (for greater accuracy I use a timer with a resolution of 100us)
#define EncodersProbeTimeInSeconds 0.02 //same value as above but as float

//(For PID_Module)
//Experimentally Designated
//Converting PWM value to speed value
//		 RealMotorSpeed = ax +b
#define A_FactorMotor 220
#define B_FactorMotor 50

//BLE_Comm
#define TimeBeetweenNextDataPart 10

//For LedModule
#define LED_TOGGLE_TIME 500

#endif //_INC_LF_AppConfig_H
