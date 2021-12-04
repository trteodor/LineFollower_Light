#include "main.h"
#include "adc.h"
#include "dma.h"

#include "LineSensorsModule.h"
#include "Encoders_Module.h"
#include "EEPROM.h"
#include "EEPROM_VarLocDef.h"

SensorModule_t SensorModule;
/*Few field of the structure above are also modified by HM10Ble App Module*/

static void Read_SensorsValue_From_EEPROM();

void SM_SensorModuleInit()
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)SensorModule.SensorADCValues,8); //Turn on Sensor Read
	Read_SensorsValue_From_EEPROM();
}

float SM_SensorsCalculateError()
{
	//I could use cross table to minimalize the function volume but it is done as is :)
	static int OST_KIER_BL=0;

	if(SensorModule.SensorADCValues[3] > SensorModule.LineDetectValue[3]
			&& SensorModule.SensorADCValues[4] > SensorModule.LineDetectValue[4]){
		SensorModule.PositionErrorValue=0;  	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[0] > SensorModule.LineDetectValue[0]
			&& SensorModule.SensorADCValues[1] > SensorModule.LineDetectValue[1]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[5];  OST_KIER_BL=1;	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[1] > SensorModule.LineDetectValue[1]
			&& SensorModule.SensorADCValues[2] > SensorModule.LineDetectValue[2]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[3];  	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[2] > SensorModule.LineDetectValue[2]
			&& SensorModule.SensorADCValues[3] > SensorModule.LineDetectValue[3]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[1];  	return SensorModule.PositionErrorValue;
	}

	if(SensorModule.SensorADCValues[7] > SensorModule.LineDetectValue[7]
			&& SensorModule.SensorADCValues[6] > SensorModule.LineDetectValue[6]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[5]; OST_KIER_BL=-1; 	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[6] > SensorModule.LineDetectValue[6]
			&& SensorModule.SensorADCValues[5] > SensorModule.LineDetectValue[5]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[3]; 	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[5] > SensorModule.LineDetectValue[5]
			&& SensorModule.SensorADCValues[4] > SensorModule.LineDetectValue[4]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[1];  	return SensorModule.PositionErrorValue;
	}


	if(SensorModule.SensorADCValues[0] > SensorModule.LineDetectValue[0]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[6];   OST_KIER_BL=1; 	 return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[1] > SensorModule.LineDetectValue[1]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[4];     return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[2] > SensorModule.LineDetectValue[2]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[2];  	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[3] > SensorModule.LineDetectValue[3]){
		SensorModule.PositionErrorValue=SensorModule.SensorErrorValue[0];  	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[4] > SensorModule.LineDetectValue[4]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[0];  	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[5] > SensorModule.LineDetectValue[5]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[2];  	return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[6] > SensorModule.LineDetectValue[6]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[4];    return SensorModule.PositionErrorValue;
	}
	if(SensorModule.SensorADCValues[7] > SensorModule.LineDetectValue[7]){
		SensorModule.PositionErrorValue=-SensorModule.SensorErrorValue[6];  OST_KIER_BL=-1;	return SensorModule.PositionErrorValue;
	}

	if (OST_KIER_BL==-1 )  {  SensorModule.PositionErrorValue=-SensorModule.SensorErrorMaxValue; 	return SensorModule.PositionErrorValue; }
	if (OST_KIER_BL==1  )  {  SensorModule.PositionErrorValue=SensorModule.SensorErrorMaxValue;  	return SensorModule.PositionErrorValue; }

	//PositionErrorValue=99; //Function Error
	return 0;
}






static void Read_SensorsValue_From_EEPROM()
{
EEPROM_READ_FLOAT(EEPROM_ErrW1_Addr, &SensorModule.SensorErrorValue[0]);
EEPROM_READ_FLOAT(EEPROM_ErrW2_Addr, &SensorModule.SensorErrorValue[1]);
EEPROM_READ_FLOAT(EEPROM_ErrW3_Addr, &SensorModule.SensorErrorValue[2]);
EEPROM_READ_FLOAT(EEPROM_ErrW4_Addr, &SensorModule.SensorErrorValue[3]);
EEPROM_READ_FLOAT(EEPROM_ErrW5_Addr, &SensorModule.SensorErrorValue[4]);
EEPROM_READ_FLOAT(EEPROM_ErrW6_Addr, &SensorModule.SensorErrorValue[5]);
EEPROM_READ_FLOAT(EEPROM_ErrW7_Addr, &SensorModule.SensorErrorValue[6]);
EEPROM_READ_FLOAT(EEPROM_ErrW_Max_Addr, &SensorModule.SensorErrorMaxValue);

EEPROM_READ_INT(EEPROM_S1_LineIsDetV_Addr, &SensorModule.LineDetectValue[0]);
EEPROM_READ_INT(EEPROM_S2_LineIsDetV_Addr, &SensorModule.LineDetectValue[1]);
EEPROM_READ_INT(EEPROM_S3_LineIsDetV_Addr, &SensorModule.LineDetectValue[2]);
EEPROM_READ_INT(EEPROM_S4_LineIsDetV_Addr, &SensorModule.LineDetectValue[3]);
EEPROM_READ_INT(EEPROM_S5_LineIsDetV_Addr, &SensorModule.LineDetectValue[4]);
EEPROM_READ_INT(EEPROM_S6_LineIsDetV_Addr, &SensorModule.LineDetectValue[5]);
EEPROM_READ_INT(EEPROM_S7_LineIsDetV_Addr, &SensorModule.LineDetectValue[6]);
EEPROM_READ_INT(EEPROM_S8_LineIsDetV_Addr, &SensorModule.LineDetectValue[7]);
}
