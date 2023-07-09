#include "main.h"
#include "adc.h"
#include "dma.h"

#include "LinePosEstimator.h"
#include "Encoders_Module.h"
#include "EEmuConfig.h"

#include <stdlib.h>

SensorModule_t SensorModule;
/*Few field of the structure above are also modified by BLE App Module*/

static void Read_SensorsValue_From_EEPROM();
static float SM_SensorsCalculateError();

void SM_Init(void)
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)SensorModule.SensorADCValues,12); //Turn on Sensor Read
	Read_SensorsValue_From_EEPROM();
}

void SM_Main(void)
{
		static float _PreviousPosErrorValue;
	float _PosErrorValue;
	static int SavedLineSide;
	static uint32_t SavedBigErrorTime; /*Big error is when _PosErrorValue > SensorErrorValue[7] */


	/*Try detect big error position*/
	for(int i=0; i<3; i++)
	{
		if(SensorModule.SensorADCValues[i] > L_DetVal)
		{
			SavedBigErrorTime = HAL_GetTick();
			SavedLineSide = LineOnRightSide;
		}
	}

	for(int i=11; i>9; i--)
	{
		if(SensorModule.SensorADCValues[i] > L_DetVal)
		{
			SavedBigErrorTime = HAL_GetTick();
			SavedLineSide = LineOnLeftSide ;
		}
	}

	if( SavedLineSide != LineSideClear && (SavedBigErrorTime + TimeMSToClearBigErrorFlag) < HAL_GetTick() )
	{
		SavedLineSide = LineSideClear;
	}
	 /*Get Actual position error */
	_PosErrorValue = SM_SensorsCalculateError();

	if(_PosErrorValue == LineNotDetectedErrorVal ) /*If line not detected */
	{

		if(  SavedLineSide == LineOnLeftSide )
		{
			SavedBigErrorTime = HAL_GetTick();
			_PosErrorValue = -SensorModule.SensorErrorMaxValue;
		}
		else if( SavedLineSide == LineOnRightSide )
		{
			SavedBigErrorTime = HAL_GetTick();
			_PosErrorValue = SensorModule.SensorErrorMaxValue;
		}
		else
		{
			_PosErrorValue = _PreviousPosErrorValue ;
			/*nothing to do*/ /*Continue with previous error */
		}
		SensorModule.PositionErrorValue =  _PosErrorValue ;

		return;
	}

	_PreviousPosErrorValue = _PosErrorValue;

	SensorModule.PositionErrorValue =  _PosErrorValue ;
	return;

}

void SM_GetAndVerifyCalculatedError()
{
	static float _PreviousPosErrorValue;
	float _PosErrorValue;
	static int SavedLineSide;
	static uint32_t SavedBigErrorTime; /*Big error is when _PosErrorValue > SensorErrorValue[7] */


	/*Try detect big error position*/
	for(int i=0; i<3; i++)
	{
		if(SensorModule.SensorADCValues[i] > L_DetVal)
		{
			SavedBigErrorTime = HAL_GetTick();
			SavedLineSide = LineOnRightSide;
		}
	}

	for(int i=11; i>9; i--)
	{
		if(SensorModule.SensorADCValues[i] > L_DetVal)
		{
			SavedBigErrorTime = HAL_GetTick();
			SavedLineSide = LineOnLeftSide ;
		}
	}

	if( SavedLineSide != LineSideClear && (SavedBigErrorTime + TimeMSToClearBigErrorFlag) < HAL_GetTick() )
	{
		SavedLineSide = LineSideClear;
	}
	 /*Get Actual position error */
	_PosErrorValue = SM_SensorsCalculateError();

	if(_PosErrorValue == LineNotDetectedErrorVal ) /*If line not detected */
	{

		if(  SavedLineSide == LineOnLeftSide )
		{
			SavedBigErrorTime = HAL_GetTick();
			_PosErrorValue = -SensorModule.SensorErrorMaxValue;
		}
		else if( SavedLineSide == LineOnRightSide )
		{
			SavedBigErrorTime = HAL_GetTick();
			_PosErrorValue = SensorModule.SensorErrorMaxValue;
		}
		else
		{
			_PosErrorValue = _PreviousPosErrorValue ;
			/*nothing to do*/ /*Continue with previous error */
		}
		SensorModule.PositionErrorValue =  _PosErrorValue ;

		return;
	}

	_PreviousPosErrorValue = _PosErrorValue;

	SensorModule.PositionErrorValue =  _PosErrorValue ;
	return;
}


/*
 * @brief
 * fun  - SM_SensorsCalculateError
 *
 * +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
 * | S1        | S2     | S3     | S4     | S5     | S6   | S7   | S8     | S9     | S10    | S11    | S12       |
 * | SideL_Max | SideL4 | SideL3 | SideL2 | SideL1 | Mid1 | Mid2 | SideR1 | SideR2 | SideR3 | SideR4 | SideR_Max |
 * +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
 */
static float SM_SensorsCalculateError(void)
{
	if( Mid1 > L_DetVal /*-ER1*/
			&& Mid2 > L_DetVal
				&& SideL1 > L_DetVal)
	{
		return SensorModule.SensorErrorValue[0];
	}
	if( Mid1 > L_DetVal /*+ER1*/
			&& Mid2 > L_DetVal
				&& SideR1 > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[0];
	}

	if( Mid1 > L_DetVal /*Error not detected*/
			&& Mid2 > L_DetVal)
	{
		return SensorModule.PositionErrorValue=0;
	}

	if( SideR2 > L_DetVal /*+ER3*/
			&& SideR1 > L_DetVal
				&& Mid2 > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[2];
	}
	if( SideL2 > L_DetVal /*-ER3*/
			&& SideL1 > L_DetVal
				&& Mid1 > L_DetVal)
	{
		return SensorModule.SensorErrorValue[2];
	}

	if( SideR1 > L_DetVal /*+ER2*/
			&& Mid2 > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[1];
	}

	if( SideL1 > L_DetVal /*-ER2*/
			&& Mid1 > L_DetVal)
	{
		return SensorModule.SensorErrorValue[1];
	}

	if( SideR1 > L_DetVal /*-ER5*/
			&& SideR2 > L_DetVal
				&& SideR3 > L_DetVal )
	{
		return -SensorModule.SensorErrorValue[4];
	}

	if( SideL1 > L_DetVal /*+ER5*/
			&& SideL2 > L_DetVal
				&& SideL3 > L_DetVal )
	{
		return SensorModule.SensorErrorValue[4];
	}

	if( SideR1 > L_DetVal /*-ER4*/
			&& SideR2 > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[3];
	}

	if( SideL1 > L_DetVal /*+ER4*/
			&& SideL2 > L_DetVal)
	{
		return SensorModule.SensorErrorValue[3];
	}

	if( SideR2 > L_DetVal /*-ER7*/
			&& SideR3 > L_DetVal
				&& SideR4 > L_DetVal )
	{
		return -SensorModule.SensorErrorValue[6];
	}

	if( SideL2> L_DetVal /*+ER7*/
			&& SideL3 > L_DetVal
				&& SideL4 > L_DetVal )
	{
		return SensorModule.SensorErrorValue[6];
	}

	if( SideR2 > L_DetVal /*-ER6*/
			&& SideR3 > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[5];
	}
	if( SideL2 > L_DetVal /*+ER6*/
			&& SideL3 > L_DetVal)
	{
		return SensorModule.SensorErrorValue[5];
	}

	if( SideR3 > L_DetVal /*-ER9*/
			&& SideR4 > L_DetVal
				&& SideR_Max > L_DetVal )
	{
		return -SensorModule.SensorErrorValue[8];
	}

	if( SideL3> L_DetVal /*+ER9*/
			&& SideL4 > L_DetVal
				&& SideL_Max  > L_DetVal )
	{
		return SensorModule.SensorErrorValue[8];
	}

	if( SideR3 > L_DetVal /*-ER8*/
			&& SideR4 > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[7];
	}

	if( SideL3 > L_DetVal /*+ER8*/
			&& SideL4 > L_DetVal)
	{
		return SensorModule.SensorErrorValue[7];
	}

	if( SideR4 > L_DetVal /*-ER10*/
			&& SideR_Max > L_DetVal)
	{
		return -SensorModule.SensorErrorValue[9];
	}

	if( SideL4 > L_DetVal /*+ER10*/
			&& SideL_Max > L_DetVal)
	{
		return SensorModule.SensorErrorValue[9];
	}

	if( SideR_Max > L_DetVal ) /*-ER11*/
	{
		return -SensorModule.SensorErrorValue[10];
	}

	if( SideL_Max > L_DetVal ) /*+ER11*/
	{
		return SensorModule.SensorErrorValue[10];
	}

	/*Line not detected */
	return LineNotDetectedErrorVal;
}
/********************************************************************************/
static void Read_SensorsValue_From_EEPROM(void)
{
//	EEPROM_READ_FLOAT(EEPROM_ErrW1_Addr, &SensorModule.SensorErrorValue[0]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW2_Addr, &SensorModule.SensorErrorValue[1]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW3_Addr, &SensorModule.SensorErrorValue[2]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW4_Addr, &SensorModule.SensorErrorValue[3]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW5_Addr, &SensorModule.SensorErrorValue[4]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW6_Addr, &SensorModule.SensorErrorValue[5]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW7_Addr, &SensorModule.SensorErrorValue[6]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW8_Addr, &SensorModule.SensorErrorValue[7]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW9_Addr, &SensorModule.SensorErrorValue[8]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW10_Addr, &SensorModule.SensorErrorValue[9]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW11_Addr, &SensorModule.SensorErrorValue[10]);
//	EEPROM_READ_FLOAT(EEPROM_ErrW_Max_Addr, &SensorModule.SensorErrorMaxValue);
	SensorModule.LineDetectValue = 3000 ;
}
