#ifndef _LineSensor_H
#define _LineSensor_H

#define LineIsDetectedDefValue 2500

#define LineNotDetectedErrorVal 999

typedef struct
{
	float PositionErrorValue;
	uint16_t SensorADCValues[12];
	float SensorErrorValue[11];
	float SensorErrorMaxValue;
	int LineDetectValue;
}SensorModule_t;
/*
+-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
| S1        | S2     | S3     | S4     | S5     | S6   | S7   | S8     | S9     | S10    | S11    | S12       |
| SideL_Max | SideL4 | SideL3 | SideL2 | SideL1 | Mid1 | Mid2 | SideR1 | SideR2 | SideR3 | SideR4 | SideR_Max |
+-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
* (-1)
 */

#define L_DetVal SensorModule.LineDetectValue

#define Mid1 SensorModule.SensorADCValues[5]
#define Mid2 SensorModule.SensorADCValues[6]

#define SideL1 SensorModule.SensorADCValues[4]
#define SideL2 SensorModule.SensorADCValues[3]
#define SideL3 SensorModule.SensorADCValues[2]
#define SideL4 SensorModule.SensorADCValues[1]
#define SideL_Max SensorModule.SensorADCValues[0]

#define SideR1 SensorModule.SensorADCValues[7]
#define SideR2 SensorModule.SensorADCValues[8]
#define SideR3 SensorModule.SensorADCValues[9]
#define SideR4 SensorModule.SensorADCValues[10]
#define SideR_Max SensorModule.SensorADCValues[11]




extern SensorModule_t SensorModule;
/*Few field of the structure above are also modified by HM10Ble App Module*/

void SM_SensorModuleInit();
void SM_GetAndVerifyCalculatedError();

#endif //_LineSensor_H
