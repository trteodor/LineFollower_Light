#ifndef _LineSensor_H
#define _LineSensor_H

#define LineIsDetectedDefValue 2500

typedef struct
{
	float PositionErrorValue;
	uint16_t SensorADCValues[12];
	float SensorErrorValue[7];
	float SensorErrorMaxValue;
	int LineDetectValue[8];
}SensorModule_t;

extern SensorModule_t SensorModule;
/*Few field of the structure above are also modified by HM10Ble App Module*/

void SM_SensorModuleInit();
float SM_SensorsCalculateError();


#endif //_LineSensor_H
