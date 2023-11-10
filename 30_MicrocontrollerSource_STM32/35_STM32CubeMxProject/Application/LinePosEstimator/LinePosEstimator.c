
#include "adc.h"
#include "dma.h"

#include "LinePosEstimator.h"
#include "EEmu.h"
#include "BluetoothClassicComm.h"
#include "EncodersHandler.h"



#define TimeMSToClearBigErrorFlag    70

// #define LineSideClear    0
// #define LineOnLeftSide   1
// #define LineOnRightSide  2


#define LINE_EVENT_CONFIRMATION_MAGIC_VALUE_MS 2 //TODO:(explanation) (10 / 2.0m/s = 5ms )
#define LINE_EVENT_MINIMUM_SENSOR_COUNT_RIGHT_ANGLE 5
#define LINE_EVENT_MINIMUM_SENSOR_COUNT_CROSS 4
#define LINE_MINIMUM_ONE_EVENT_DIFF_DIST 0.05F //TODO: 0.05 [m](at least 5cm per one event In my judgement)

#define LINE_NOT_DETECTED_MAGIC_NUMBER        999
#define LINE_DETECTED_ADC_VALUE               3000
#define LINE_SENSOR_DATA_COUNT 				  12

#define LF_DRIV_SRIGHT_MEAS_PERIOD_MS         20

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
	uint16_t LeftLinePosConfidence;
	uint16_t RightLinePosConfidence;
	bool isThemeBlackFlag;
}LineEstimatorDesc_t;

static LineEstimatorDesc_t LineEstimator;

float NVM_StrghtLineMinLght;
float NVM_StrghtLineMaxYawRate;

static void (*LineEventCorruptedCallBack_p)(LinePosEstimatorEvent_t LinePosEstEv);
static void (*DrivingAtStrightLineCallBack_p)(void);

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

typedef enum{
	LeftMax,
	LeftW4,
	LeftW3,
	LeftW2,
	LeftW1,
	MidL,
	MidR,
	RightW1,
	RightW2,
	RightW3,
	RightW4,
	RightMax
}ErrPosition_e;


#define Mid1 LineEstimator.LineSensorsADCVal[MidL]
#define Mid2 LineEstimator.LineSensorsADCVal[MidR]
#define SideL1 LineEstimator.LineSensorsADCVal[LeftW1]
#define SideL2 LineEstimator.LineSensorsADCVal[LeftW2]
#define SideL3 LineEstimator.LineSensorsADCVal[LeftW3]
#define SideL4 LineEstimator.LineSensorsADCVal[LeftW4]
#define SideL_Max LineEstimator.LineSensorsADCVal[LeftMax]
#define SideR1 LineEstimator.LineSensorsADCVal[RightW1]
#define SideR2 LineEstimator.LineSensorsADCVal[RightW2]
#define SideR3 LineEstimator.LineSensorsADCVal[RightW3]
#define SideR4 LineEstimator.LineSensorsADCVal[RightW4]
#define SideR_Max LineEstimator.LineSensorsADCVal[RightMax]
/*Extra Macros END*/


/**********************************************************************/
static void GetErrorWeightsFromNvm();

static float EstimatePositionError();

void LPE_RegisterLineEventCallBack(void LineEvCallback(LinePosEstimatorEvent_t LinePosEstEv) )
{
	LineEventCorruptedCallBack_p = LineEvCallback;
}

void LPE_RegisterDrivingAtStrightLineCallBack(void DrivStrightLineEvCallback(void) )
{
DrivingAtStrightLineCallBack_p = DrivStrightLineEvCallback;
}



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

static void GetTypeOfThemeFromNvm(void)
{
	EE_ReadVariableU32(EE_NvmAddr_BlackThemeFlag_U32,(uint32_t *)&LineEstimator.isThemeBlackFlag);
}

static void GetStrghtLineParams(void)
{
	EE_ReadVariableF32(EE_NvmAddr_StrghtLineMinLght_F32,&NVM_StrghtLineMinLght);
	EE_ReadVariableF32(EE_NvmAddr_StrghtLineMaxYawRate_F32,&NVM_StrghtLineMaxYawRate);
}

/**********************************************************************/
static void BleUpdateNvmDataCallBack(void)
{
	GetErrorWeightsFromNvm();
	GetTypeOfThemeFromNvm();
	GetStrghtLineParams();
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
	if(false == LineEstimator.isThemeBlackFlag)
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
	}
	else{
		/*white line on black theme*/
				/************************************************/
		if( Mid1 < LINE_DETECTED_ADC_VALUE /*-ER2*/
					&& SideL1 < LINE_DETECTED_ADC_VALUE){
			return LineEstimator.ErrorWeightValueTable[1];
		}

		if(  Mid2 < LINE_DETECTED_ADC_VALUE/*+ER2*/
					&& SideR1 < LINE_DETECTED_ADC_VALUE){
			return -LineEstimator.ErrorWeightValueTable[1];
		}
		/************************************************/

		/************************************************/
		if( Mid1 < LINE_DETECTED_ADC_VALUE /*Error not detected*/
				&& Mid2 < LINE_DETECTED_ADC_VALUE){
			return LineEstimator.PositionErrorValue=0;
		}
		/************************************************/

		if( Mid2 < LINE_DETECTED_ADC_VALUE /*+ER1*/){
			return -LineEstimator.ErrorWeightValueTable[0];
		}
		if( Mid1 < LINE_DETECTED_ADC_VALUE /*-ER1*/){
			return LineEstimator.ErrorWeightValueTable[0];
		}


		if( SideR2 < LINE_DETECTED_ADC_VALUE /*+ER4*/
				&& SideR1 < LINE_DETECTED_ADC_VALUE){
			return -LineEstimator.ErrorWeightValueTable[3];
		}
		if( SideL2 < LINE_DETECTED_ADC_VALUE /*-ER4*/
				&& SideL1 < LINE_DETECTED_ADC_VALUE){
			return LineEstimator.ErrorWeightValueTable[3];
		}

		/************************************************/
		if( SideR1 < LINE_DETECTED_ADC_VALUE /*+ER3*/){
			return -LineEstimator.ErrorWeightValueTable[2];
		}
		if( SideL1 < LINE_DETECTED_ADC_VALUE /*-ER3*/){
			return LineEstimator.ErrorWeightValueTable[2];
		}
		/************************************************/


		if( SideR3 < LINE_DETECTED_ADC_VALUE /*+ER6*/
				&& SideR2 < LINE_DETECTED_ADC_VALUE){
			return -LineEstimator.ErrorWeightValueTable[5];
		}
		if( SideL3 < LINE_DETECTED_ADC_VALUE /*-ER6*/
				&& SideL2 < LINE_DETECTED_ADC_VALUE){
			return LineEstimator.ErrorWeightValueTable[5];
		}

		if( SideR2 < LINE_DETECTED_ADC_VALUE /*+ER5*/){
			return -LineEstimator.ErrorWeightValueTable[4];
		}
		if( SideL2 < LINE_DETECTED_ADC_VALUE /*-ER5*/){
			return LineEstimator.ErrorWeightValueTable[4];
		}


		if( SideR4 < LINE_DETECTED_ADC_VALUE /*+ER8*/
				&& SideR3 < LINE_DETECTED_ADC_VALUE){
			return -LineEstimator.ErrorWeightValueTable[7];
		}
		if( SideL4 < LINE_DETECTED_ADC_VALUE /*-ER8*/
				&& SideL3 < LINE_DETECTED_ADC_VALUE){
			return LineEstimator.ErrorWeightValueTable[7];
		}


		if( SideR3 < LINE_DETECTED_ADC_VALUE /*+ER7*/){
			return -LineEstimator.ErrorWeightValueTable[6];
		}
		if( SideL3 < LINE_DETECTED_ADC_VALUE /*-ER7*/){
			return LineEstimator.ErrorWeightValueTable[6];
		}


		if( SideR_Max < LINE_DETECTED_ADC_VALUE /*+ER10*/
				&& SideR4 < LINE_DETECTED_ADC_VALUE){
			return -LineEstimator.ErrorWeightValueTable[9];
		}
		if( SideL_Max < LINE_DETECTED_ADC_VALUE /*-ER10*/
				&& SideL4 < LINE_DETECTED_ADC_VALUE){
			return LineEstimator.ErrorWeightValueTable[9];
		}

		if( SideR4 < LINE_DETECTED_ADC_VALUE /*+ER9*/){
			return -LineEstimator.ErrorWeightValueTable[8];
		}
		if( SideL4 < LINE_DETECTED_ADC_VALUE /*-ER9*/){
			return LineEstimator.ErrorWeightValueTable[8];
		}

		if( SideR_Max < LINE_DETECTED_ADC_VALUE /*+ER11*/){
			return -LineEstimator.ErrorWeightValueTable[10];
		}
		if( SideL_Max < LINE_DETECTED_ADC_VALUE /*-ER11*/){
			return LineEstimator.ErrorWeightValueTable[10];
		}
	}


	/*Line not detected */
	return LINE_NOT_DETECTED_MAGIC_NUMBER;
}






/** @brief TryDetectStrigthLine(LinePosEstimatorEvent_t LinePosEstEv, float CurrErrPosValue)
* @details 
* @return  
*/
void TryDetectStrigthLine(LinePosEstimatorEvent_t LinePosEstEv, float CurrErrPosValue)
{

	static uint32_t LogStrightLineTimer = 0;
	static uint32_t MesaureTimerPeriod = 0; 
	static float TimeStrightLineYrStart;
	static float DistanceStrightLineYrStart;
	static float DistanceStrightLineYrEnd;
	static bool PreviousStateIsStrightLine = false;
	static bool CurrentStateIsStrightLine = false;
	static uint32_t LastTimeOfBigError = 0;
	static bool DetectedDrivAtStrightLineCommitedFlag = false;
	float yawRateValue = ENC_GetYawRateWhBased();

	/**/
	CurrentStateIsStrightLine = PreviousStateIsStrightLine;
	/**/

	if(fabs(CurrErrPosValue) > fabs(LineEstimator.ErrorWeightValueTable[8]) )
	{
		LastTimeOfBigError = HAL_GetTick();
		CurrentStateIsStrightLine = false;
	}

	if( ( (HAL_GetTick() - MesaureTimerPeriod) > LF_DRIV_SRIGHT_MEAS_PERIOD_MS) 
								&& (HAL_GetTick() - LastTimeOfBigError > LF_DRIV_SRIGHT_MEAS_PERIOD_MS) )
	{
		if(fabs(yawRateValue) < fabs(NVM_StrghtLineMaxYawRate) )
		{
			if(PreviousStateIsStrightLine == false)
			{
				DetectedDrivAtStrightLineCommitedFlag = false;
				TimeStrightLineYrStart = HAL_GetTick();
				DistanceStrightLineYrStart = ENC_GetTravelledDistance();
				CurrentStateIsStrightLine = true;
				LogStrightLineTimer = HAL_GetTick();
			}
		}
		else
		{
			if(PreviousStateIsStrightLine == true)
			{
				DetectedDrivAtStrightLineCommitedFlag = false;
				DistanceStrightLineYrEnd = ENC_GetTravelledDistance();
				CurrentStateIsStrightLine = false;

				LogStrightLineTimer = HAL_GetTick();
			}
		}
	}

	if( (PreviousStateIsStrightLine == true) && (CurrentStateIsStrightLine == false) )
	{
		DetectedDrivAtStrightLineCommitedFlag = false;
		if( fabs(DistanceStrightLineYrEnd - DistanceStrightLineYrStart) > NVM_StrghtLineMinLght)
		{
			BLU_DbgMsgTransmit("Leaving StrightLine: TrvDSt: %.3f, TrvDend:%.3f",DistanceStrightLineYrStart ,DistanceStrightLineYrEnd );
		}
	}

	if( ( ( fabs( ENC_GetTravelledDistance() - DistanceStrightLineYrStart) ) > NVM_StrghtLineMinLght )  //m
			&& (CurrentStateIsStrightLine == true) && (PreviousStateIsStrightLine == true)
			&& (fabs( ENC_GetVehicleSpeed() ) > 0.2F) ) //m/s
	{
		/*Veh driving at stright line*/
		if(false == DetectedDrivAtStrightLineCommitedFlag)
		{
			if(DrivingAtStrightLineCallBack_p != NULL){
				DrivingAtStrightLineCallBack_p();
			}
			BLU_DbgMsgTransmit("VehAtStrightLine: AtLeastBy: %f[m]",NVM_StrghtLineMinLght);
			DetectedDrivAtStrightLineCommitedFlag = true;
		}

		if( (HAL_GetTick() - LogStrightLineTimer) > 1000)
		{
			LogStrightLineTimer = HAL_GetTick();
			(void)TimeStrightLineYrStart;
			BLU_DbgMsgTransmit("VehAtStrightLine: TrvDStart: %.3f ucTStamp: %d CurrD: %.3f",DistanceStrightLineYrStart,TimeStrightLineYrStart, ENC_GetTravelledDistance() );
		}
	}

	PreviousStateIsStrightLine = CurrentStateIsStrightLine;
}




LinePosEstimatorEvent_t TryDetectRightAnglesAndCrosses(void)
{
	uint32_t CrossLineDetCounterLeft = 0;
	uint32_t CrossLineDetCounterRight = 0;
	LinePosEstimatorEvent_t LinePosEstimatorEvent = Event_None;

	static bool AlreadyOnCrossOrRightAngle = false;

	// static uint32_t isOnCrossProbingTimer = 0;
	// static bool isOnCrossProbingFlag =false;

	static bool isOnRightRightAngleProbingFlag=false;
	static bool isOnLeftRightAngleProbingFlag =false;
	static uint32_t isOnRightRightAngleProbingTimer = 0;
	static uint32_t isOnLeftRightAngleProbingTimer = 0;

	float RequiredConfirmationMagicValue =  LINE_EVENT_CONFIRMATION_MAGIC_VALUE_MS / ENC_GetVehicleSpeed();
	if(RequiredConfirmationMagicValue > 50.0F)
	{
		RequiredConfirmationMagicValue  = 50.0F;
	}
	uint32_t RequiredConfirmationMagicValue_u32 = (uint32_t)RequiredConfirmationMagicValue;


	static float travelledDistanceEvent = 0.0F;
	float travelledDistanceCurr = ENC_GetTravelledDistance();

	if(travelledDistanceCurr == 0.0F)
	{
		travelledDistanceEvent = 0.0F;
	}

	static uint32_t lastEventDetectedTimer = 0;


	for(int i=LeftMax; i<MidL; i++)
	{
		if(LineEstimator.LineSensorsADCVal[i] > LINE_DETECTED_ADC_VALUE)
		{
			CrossLineDetCounterRight++;
		}
	}

	for(int i=RightMax; i>MidR; i--)
	{
		if(LineEstimator.LineSensorsADCVal[i] > LINE_DETECTED_ADC_VALUE)
		{
			CrossLineDetCounterLeft++;
		}
	}

	if(CrossLineDetCounterLeft == 0 && CrossLineDetCounterRight == 0 && AlreadyOnCrossOrRightAngle == true)
	{
		AlreadyOnCrossOrRightAngle = false;
		travelledDistanceEvent= travelledDistanceCurr;
		lastEventDetectedTimer = HAL_GetTick();
	}


	if( ( fabs(travelledDistanceCurr - travelledDistanceEvent) > LINE_MINIMUM_ONE_EVENT_DIFF_DIST) && (HAL_GetTick() - lastEventDetectedTimer > 20 ) )
	{
		if(CrossLineDetCounterLeft >= LINE_EVENT_MINIMUM_SENSOR_COUNT_RIGHT_ANGLE && CrossLineDetCounterRight <= 1)
		{
			if(false == isOnLeftRightAngleProbingFlag)
			{
				isOnLeftRightAngleProbingFlag = true;
				isOnLeftRightAngleProbingTimer = HAL_GetTick();

			}
			else
			{
				if( HAL_GetTick() - isOnLeftRightAngleProbingTimer > RequiredConfirmationMagicValue_u32)
				{
					isOnLeftRightAngleProbingTimer = HAL_GetTick();
					isOnLeftRightAngleProbingFlag = false;
					travelledDistanceEvent = travelledDistanceCurr;
					lastEventDetectedTimer = HAL_GetTick();
					LinePosEstimatorEvent = Event_LeftRightAngle;
					// BLU_DbgMsgTransmit("Left|| RightAngleDetected TrvD: %.3f",travelledDistanceCurr);
				}
			}
		}
		else if(CrossLineDetCounterRight >= LINE_EVENT_MINIMUM_SENSOR_COUNT_RIGHT_ANGLE  && CrossLineDetCounterLeft  <= 1)
		{
			if(false == isOnRightRightAngleProbingFlag)
			{
				isOnRightRightAngleProbingFlag = true;
				isOnRightRightAngleProbingTimer = HAL_GetTick();

			}
			else
			{
				if( HAL_GetTick() - isOnRightRightAngleProbingTimer > RequiredConfirmationMagicValue_u32)
				{
					isOnRightRightAngleProbingTimer = HAL_GetTick();
					isOnRightRightAngleProbingFlag = false;
					travelledDistanceEvent = travelledDistanceCurr;
					lastEventDetectedTimer = HAL_GetTick();
					LinePosEstimatorEvent = Event_RightRightAngle;
					// BLU_DbgMsgTransmit("Right|| RightAngleDetected TrvD: %.3f",travelledDistanceCurr);
				}
			}
		}
	}
	else
	{
		isOnRightRightAngleProbingFlag = false;
		isOnLeftRightAngleProbingFlag = false;
	}

	return LinePosEstimatorEvent;
}

LinePostionEnum_t LinePositionEstimator(void)
{
	static float _PreviousPosErrorValue;
	float _PosErrorValue;
	static int SavedLineSide;
	static uint32_t SavedBigErrorTime; /*Big error is when _PosErrorValue > ErrorWeightValueTable[7] */

	if(false == LineEstimator.isThemeBlackFlag)
	{
				/*Try detect big error position*/
		for(int i=LeftMax; i<LeftW2; i++)
		{
			if(LineEstimator.LineSensorsADCVal[i] > LINE_DETECTED_ADC_VALUE)
			{
				SavedBigErrorTime = HAL_GetTick();
				SavedLineSide = LineOnRightSide;
			}
		}

		for(int i=RightMax; i>RightW2; i--)
		{
			if(LineEstimator.LineSensorsADCVal[i] > LINE_DETECTED_ADC_VALUE)
			{
				SavedBigErrorTime = HAL_GetTick();
				SavedLineSide = LineOnLeftSide ;
			}
		}
	}
	else{
		/*Try detect big error position*/
		for(int i=LeftMax; i<LeftW2; i++)
		{
			if(LineEstimator.LineSensorsADCVal[i] < LINE_DETECTED_ADC_VALUE)
			{
				SavedBigErrorTime = HAL_GetTick();
				SavedLineSide = LineOnRightSide;
			}
		}

		for(int i=RightMax; i>RightW2; i--)
		{
			if(LineEstimator.LineSensorsADCVal[i] < LINE_DETECTED_ADC_VALUE)
			{
				SavedBigErrorTime = HAL_GetTick();
				SavedLineSide = LineOnLeftSide ;
			}
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
	GetTypeOfThemeFromNvm();
	BLU_RegisterNvMdataUpdateInfoCallBack( (void *)BleUpdateNvmDataCallBack);
}


/*Expected call period <= 1ms*/
void LPE_Task(void)
{
	LinePosEstimatorEvent_t LinePosEstEv = Event_None;

	LinePositionEstimator();
	SendLinePosEstDataReportForBle();
	LinePosEstEv = TryDetectRightAnglesAndCrosses();
	if(LinePosEstEv != Event_None && LineEventCorruptedCallBack_p != NULL){
		LineEventCorruptedCallBack_p(LinePosEstEv);
	}

	TryDetectStrigthLine(LinePosEstEv,LineEstimator.PositionErrorValue);
}

float LPE_GetPosError(void)
{
	return LineEstimator.PositionErrorValue;
}


/********************************************************************************/

