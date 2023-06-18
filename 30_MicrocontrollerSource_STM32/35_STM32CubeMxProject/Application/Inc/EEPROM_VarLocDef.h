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
#define EEPROM_ErrW8_Addr 135 /*Cell 110 dont work correctly?*/
#define EEPROM_ErrW9_Addr 115
#define EEPROM_ErrW10_Addr 120
#define EEPROM_ErrW11_Addr 140 /*Cell 125 also?*/
#define EEPROM_ErrW_Max_Addr 130

#define EEPROM_IrSensorState_Addr 450

#define EEPROM_LedModeState_Addr 455

#define EEPROM_TryDetectEndLineMark_Addr 460


#endif
