/*
 *  Created on: 10.10.2021
 *      Author: Teodor
*/


#ifndef EEPROM_VarLocDef_H_
#define EEPROM_VarLocDef_H_

/*@@@@@ @@@@  EEPROM Location Definition*/

#define EEPROM_Addr_PID_Kp 50
#define EEPROM_Addr_PID_Kd 55
#define EEPROM_AddrBaseMotorSpeedValue 60
#define EEPROM_Addr_PID_Ki 65
#define EEPROM_PID_KdProbeTime_Addr 420

#define EEPROM_ErrW1_Addr 70
#define EEPROM_ErrW2_Addr 75
#define EEPROM_ErrW3_Addr 80
#define EEPROM_ErrW4_Addr 85
#define EEPROM_ErrW5_Addr 90
#define EEPROM_ErrW6_Addr 100
#define EEPROM_ErrW7_Addr 105
#define EEPROM_ErrW_Max_Addr 115


#define EEPROM_S1_LineIsDetV_Addr 265
#define EEPROM_S2_LineIsDetV_Addr 245
#define EEPROM_S3_LineIsDetV_Addr 210
#define EEPROM_S4_LineIsDetV_Addr 215
#define EEPROM_S5_LineIsDetV_Addr 220
#define EEPROM_S6_LineIsDetV_Addr 225
#define EEPROM_S7_LineIsDetV_Addr 230
#define EEPROM_S8_LineIsDetV_Addr 235


#define EEPROM_IrSensorState_Addr 450

#define EEPROM_LedModeState_Addr 455

#define EEPROM_TryDetectEndLineMark_Addr 460


#endif
