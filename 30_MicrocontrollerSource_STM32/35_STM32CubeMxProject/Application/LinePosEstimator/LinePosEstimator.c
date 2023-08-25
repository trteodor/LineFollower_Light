
#include "adc.h"
#include "dma.h"

#include "LinePosEstimator.h"
#include "EEmu.h"
#include "BluetoothClassicComm.h"
#include "EncodersHandler.h"


/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

#define LINE_NOT_DETECTED_MAGIC_NUMBER        999
#define LINE_DETECTED_ADC_VALUE               3000
#define LINE_SENSOR_DATA_COUNT 				  12

/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
typedef enum
{
	LineSideClear,
	LineOnLeftSide,
	LineOnRightSide,
}LinePostionEnum_t;



typedef struct
{
	float PositionErrorValue;
	uint16_t LineSensorsADCVal[12];
	float ErrorWeightValueTable[11];
	float SensorErrorMaxValue;
	int LineDetectValue;
}LineEstimatorDesc_t;

static LineEstimatorDesc_t LineEstimator;




/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

/*
 * @brief
 * Extra macros to increase readability
 *
 * +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
 * | S1        | S2     | S3     | S4     | S5     | S6   | S7   | S8     | S9     | S10    | S11    | S12       |
 * | SideL_Max | SideL4 | SideL3 | SideL2 | SideL1 | Mid1 | Mid2 | SideR1 | SideR2 | SideR3 | SideR4 | SideR_Max |
 * +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
 */
/*Extra Macros start*/
#define Mid1 LineEstimator.LineSensorsADCVal[5]
#define Mid2 LineEstimator.LineSensorsADCVal[6]
#define SideL1 LineEstimator.LineSensorsADCVal[4]
#define SideL2 LineEstimator.LineSensorsADCVal[3]
#define SideL3 LineEstimator.LineSensorsADCVal[2]
#define SideL4 LineEstimator.LineSensorsADCVal[1]
#define SideL_Max LineEstimator.LineSensorsADCVal[0]
#define SideR1 LineEstimator.LineSensorsADCVal[7]
#define SideR2 LineEstimator.LineSensorsADCVal[8]
#define SideR3 LineEstimator.LineSensorsADCVal[9]
#define SideR4 LineEstimator.LineSensorsADCVal[10]
#define SideR_Max LineEstimator.LineSensorsADCVal[11]
/*Extra Macros END*/

#define TimeMSToClearBigErrorFlag 30

/**********************************************************************/
static void GetErrorWeightsFromNvm();
static float EstimatePositionError();

static uint16_t MapValueToUint8_Range(uint16_t InputValue)
{
	uint16_t output;

	float input = (float)InputValue;

	float output_start = 0.0F;
	float output_end = 255.0F;

	float input_end = 4096.0F;
	float input_start = 0.0F;

	output = output_start + ( ((output_end - output_start) / (input_end - input_start)) * (input - input_start) );
	
	return (uint16_t)output;
}


/**********************************************************************/
static void GetErrorWeightsFromNvm(void)
{
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth1_F32,&LineEstimator.ErrorWeightValueTable[0]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth2_F32,&LineEstimator.ErrorWeightValueTable[1]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth3_F32,&LineEstimator.ErrorWeightValueTable[2]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth4_F32,&LineEstimator.ErrorWeightValueTable[3]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth5_F32,&LineEstimator.ErrorWeightValueTable[4]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth6_F32,&LineEstimator.ErrorWeightValueTable[5]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth7_F32,&LineEstimator.ErrorWeightValueTable[6]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth8_F32,&LineEstimator.ErrorWeightValueTable[7]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth9_F32,&LineEstimator.ErrorWeightValueTable[8]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth10_F32,&LineEstimator.ErrorWeightValueTable[9]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth11_F32,&LineEstimator.ErrorWeightValueTable[10]);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigthMax_F32,&LineEstimator.SensorErrorMaxValue);
}

/**********************************************************************/
static void BleUpdateNvmDataCallBack(void)
{
	GetErrorWeightsFromNvm();
}
/**********************************************************************/

 /*
 * +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
 * | S1        | S2     | S3     | S4     | S5     | S6   | S7   | S8     | S9     | S10    | S11    | S12       |
 * | SideL_Max | SideL4 | SideL3 | SideL2 | SideL1 | Mid1 | Mid2 | SideR1 | SideR2 | SideR3 | SideR4 | SideR_Max |
 * +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
 */
static float EstimatePositionError(void)
{

	/************************************************/
	if( Mid1 > LINE_DETECTED_ADC_VALUE /*-ER2*/
				&& SideL1 > LINE_DETECTED_ADC_VALUE){
		return LineEstimator.ErrorWeightValueTable[1];
	}

	if(  Mid2 > LINE_DETECTED_ADC_VALUE/*+ER2*/
				&& SideR1 > LINE_DETECTED_ADC_VALUE){
		return -LineEstimator.ErrorWeightValueTable[1];
	}
	/************************************************/

	/************************************************/
	if( Mid1 > LINE_DETECTED_ADC_VALUE /*Error not detected*/
			&& Mid2 > LINE_DETECTED_ADC_VALUE){
		return LineEstimator.PositionErrorValue=0;
	}
	/************************************************/

	if( Mid2 > LINE_DETECTED_ADC_VALUE /*+ER1*/){
		return -LineEstimator.ErrorWeightValueTable[0];
	}
	if( Mid1 > LINE_DETECTED_ADC_VALUE /*-ER1*/){
		return LineEstimator.ErrorWeightValueTable[0];
	}


	if( SideR2 > LINE_DETECTED_ADC_VALUE /*+ER4*/
			&& SideR1 > LINE_DETECTED_ADC_VALUE){
		return -LineEstimator.ErrorWeightValueTable[3];
	}
	if( SideL2 > LINE_DETECTED_ADC_VALUE /*-ER4*/
			&& SideL1 > LINE_DETECTED_ADC_VALUE){
		return LineEstimator.ErrorWeightValueTable[3];
	}

	/************************************************/
	if( SideR1 > LINE_DETECTED_ADC_VALUE /*+ER3*/){
		return -LineEstimator.ErrorWeightValueTable[2];
	}
	if( SideL1 > LINE_DETECTED_ADC_VALUE /*-ER3*/){
		return LineEstimator.ErrorWeightValueTable[2];
	}
	/************************************************/


	if( SideR3 > LINE_DETECTED_ADC_VALUE /*+ER6*/
			&& SideR2 > LINE_DETECTED_ADC_VALUE){
		return -LineEstimator.ErrorWeightValueTable[5];
	}
	if( SideL3 > LINE_DETECTED_ADC_VALUE /*-ER6*/
			&& SideL2 > LINE_DETECTED_ADC_VALUE){
		return LineEstimator.ErrorWeightValueTable[5];
	}

	if( SideR2 > LINE_DETECTED_ADC_VALUE /*+ER5*/){
		return -LineEstimator.ErrorWeightValueTable[4];
	}
	if( SideL2 > LINE_DETECTED_ADC_VALUE /*-ER5*/){
		return LineEstimator.ErrorWeightValueTable[4];
	}


	if( SideR4 > LINE_DETECTED_ADC_VALUE /*+ER8*/
			&& SideR3 > LINE_DETECTED_ADC_VALUE){
		return -LineEstimator.ErrorWeightValueTable[7];
	}
	if( SideL4 > LINE_DETECTED_ADC_VALUE /*-ER8*/
			&& SideL3 > LINE_DETECTED_ADC_VALUE){
		return LineEstimator.ErrorWeightValueTable[7];
	}


	if( SideR3 > LINE_DETECTED_ADC_VALUE /*+ER7*/){
		return -LineEstimator.ErrorWeightValueTable[6];
	}
	if( SideL3 > LINE_DETECTED_ADC_VALUE /*-ER7*/){
		return LineEstimator.ErrorWeightValueTable[6];
	}


	if( SideR_Max > LINE_DETECTED_ADC_VALUE /*+ER10*/
			&& SideR4 > LINE_DETECTED_ADC_VALUE){
		return -LineEstimator.ErrorWeightValueTable[9];
	}
	if( SideL_Max > LINE_DETECTED_ADC_VALUE /*-ER10*/
			&& SideL4 > LINE_DETECTED_ADC_VALUE){
		return LineEstimator.ErrorWeightValueTable[9];
	}

	if( SideR4 > LINE_DETECTED_ADC_VALUE /*+ER9*/){
		return -LineEstimator.ErrorWeightValueTable[8];
	}
	if( SideL4 > LINE_DETECTED_ADC_VALUE /*-ER9*/){
		return LineEstimator.ErrorWeightValueTable[8];
	}

	if( SideR_Max > LINE_DETECTED_ADC_VALUE /*+ER11*/){
		return -LineEstimator.ErrorWeightValueTable[10];
	}
	if( SideL_Max > LINE_DETECTED_ADC_VALUE /*-ER11*/){
		return LineEstimator.ErrorWeightValueTable[10];
	}

	/*
	* +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
	* | S1        | S2     | S3     | S4     | S5     | S6   | S7   | S8     | S9     | S10    | S11    | S12       |
	* | SideL_Max | SideL4 | SideL3 | SideL2 | SideL1 | Mid1 | Mid2 | SideR1 | SideR2 | SideR3 | SideR4 | SideR_Max |
	* +-----------+--------+--------+--------+--------+------+------+--------+--------+--------+--------+-----------+
	*/

	/*Line not detected */
	return LINE_NOT_DETECTED_MAGIC_NUMBER;
}


void TryDetectRightAngle(void)
{



}

#define LF_DRIVING_STRIGHT_YAW_RATE_VALUE 0.4

/** @brief TryDetectStrigthLine
* @details 
* @return  
*/
void TryDetectStrigthLine(void)
{
	const uint32_t MeasurePeriod = 20;
	float yawRateValue = ENC_GetYawRateWhBased();
	float travelledDistance = ENC_GetTravelledDistance();
	float posError = LineEstimator.PositionErrorValue;

	static float travelledDistanceYrStart;
	static float travelledDistanceTrEnd;
	static float travelledDistancePosErrStart;
	static float travelledDistancePosErrEnd;

	uint32_t counter;
	uint32_t timerErrorBased;
	uint32_t timerYawRateBasedStart;

	if(yawRateValue < LF_DRIVING_STRIGHT_YAW_RATE_VALUE)
	{
		
	}

}

LinePostionEnum_t LinePositionEstimator(void)
{
	static float _PreviousPosErrorValue;
	float _PosErrorValue;
	static int SavedLineSide;
	static uint32_t SavedBigErrorTime; /*Big error is when _PosErrorValue > ErrorWeightValueTable[7] */


		/*Try detect big error position*/
	for(int i=0; i<3; i++)
	{
		if(LineEstimator.LineSensorsADCVal[i] > LINE_DETECTED_ADC_VALUE)
		{
			SavedBigErrorTime = HAL_GetTick();
			SavedLineSide = LineOnRightSide;
		}
	}

	for(int i=11; i>9; i--)
	{
		if(LineEstimator.LineSensorsADCVal[i] > LINE_DETECTED_ADC_VALUE)
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
	_PosErrorValue = EstimatePositionError();

	if(_PosErrorValue == LINE_NOT_DETECTED_MAGIC_NUMBER) /*If line not detected */
	{

		if(  SavedLineSide == LineOnLeftSide )
		{
			SavedBigErrorTime = HAL_GetTick();
			_PosErrorValue = -LineEstimator.SensorErrorMaxValue;
		}
		else if( SavedLineSide == LineOnRightSide )
		{
			SavedBigErrorTime = HAL_GetTick();
			_PosErrorValue = LineEstimator.SensorErrorMaxValue;
		}
		else
		{
			_PosErrorValue = _PreviousPosErrorValue ;
			/*nothing to do*/ /*Continue with previous error */
		}
		LineEstimator.PositionErrorValue =  _PosErrorValue ;

	}

	_PreviousPosErrorValue = _PosErrorValue;

	LineEstimator.PositionErrorValue =  _PosErrorValue ;

	return LineSideClear;
}

void SendLinePosEstDataReportForBle(void)
{
	BLU_SensorDataReport_t SensorDataReport ={0};

	for(int i=0; i<LINE_SENSOR_DATA_COUNT; i++)
	{
		SensorDataReport.SensorData[i] = MapValueToUint8_Range(LineEstimator.LineSensorsADCVal[(LINE_SENSOR_DATA_COUNT -1) - i]);
	}

	SensorDataReport.PosError = LineEstimator.PositionErrorValue;
	SensorDataReport.LastLeftLinePosConfidence = 0;
	SensorDataReport.LastRightLinePosConfidence = 0;
	
	BLU_ReportSensorData(&SensorDataReport);
}

/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
void LPE_Init(void)
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)LineEstimator.LineSensorsADCVal,12); //Turn on Sensor Read
	GetErrorWeightsFromNvm();
	BLU_RegisterNvMdataUpdateInfoCallBack( (void *)BleUpdateNvmDataCallBack);
}



void LPE_Task(void)
{
	LinePositionEstimator();
	SendLinePosEstDataReportForBle();

	// BLU_DbgMsgTransmit("MappedValue 3000: %d", MapValueToUint8_Range(3000) );

}

float LPE_GetPosError(void)
{
	return LineEstimator.PositionErrorValue;
}


/********************************************************************************/

