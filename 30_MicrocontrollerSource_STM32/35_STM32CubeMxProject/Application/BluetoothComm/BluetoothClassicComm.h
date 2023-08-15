/**
 * @file BluetoothClassicComm.h
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth module to communicate wireless with Line Follower
 * I used module HC-06 - if you are interested about information of the modules please find data sheets in internet.
 */





#ifndef _BLE_ServiceModule_H
#define _BLE_ServiceModule_H

#include "stdint.h"
#include "stdbool.h"

#include "stdio.h"
#include "stdint.h"

#include "stdbool.h"
#include <stdarg.h>
#include <string.h>


/*
 * Defines configuration start
 * */

#define BLU_TRANSMIT_RING_BUFFER_SIZE     100
#define BLU_RECEIVE_RING_BUFFER_SIZE     100



/*
 * Defines configuration end
 * */

/*
 * Type defs
 * */
typedef enum
{
	BLU_Ok,
	BLU_Error
}BLU_CallStatus_t;

/*
 * Type definition of Common Header or message ID for Embedded software and desktop application
 * */
typedef enum
{
    BLU_None = 0,
    BLU_ConfirmationTag,
    BLU_DebugMessage,

    BLU_RobotStart,
    BLU_RobotStop,
	
    BLU_SimulatorStart,
    BLU_TrueBaseLoggingStart,
    BLU_SimuAndTrueDataLoggingStop,

    BLU_CommunicationStats,
    BLU_BaseDataReport,

    BLU_NvM_ErrWeigthSensorDataReq,
    BLU_NvM_ErrWeigthSensorData,

    BLU_NvM_LinePidRegDataReq,
    BLU_NvM_LinePidRegData,

    BLU_NvM_VehCfgReq,
    BLU_NvM_VehCfgData,

	BLU_NvM_MotorsFactorsReq,
    BLU_NvM_MotorsFactorsData,

	BLU_NvM_EncoderModCfgReq,
    BLU_NvM_EncoderModCfgData,

	BLU_NvM_ManualCntrlCommand,/* Virutal analog controller frame */

    BLU_SetNewRobotName,

}BLU_MessageID_t;



/*
 * Modules should report the newest data then BLE module will transmit it
 * */


/////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	float WhLftSp;
	float WhRhtSp;
	float YawRate;
	float PosX;
	float PosY;
    float PosO;
	float TravelledDistance;
}__attribute__((__packed__)) BLU_MapDataReport_t; /*Current size 6*4 = 28*/

typedef struct
{
	uint8_t SensorData[12];
	float PosError;
	uint8_t LastLeftLinePosConfidence;
	uint8_t LastRightLinePosConfidence;
}__attribute__((__packed__)) BLU_SensorDataReport_t;  /*Current size= 12+4+1+1 = 18*/

typedef struct
{
    float LinePidRegVal;
}__attribute__((__packed__)) BLE_PidRegData_t; /*Current size= 4*/

typedef struct
{
    uint8_t SyncId;
    uint32_t ucTimeStamp; //5
    BLU_MapDataReport_t CurrMapData; /*33*/
    BLU_SensorDataReport_t CurrSensorData; /*51*/
    BLE_PidRegData_t LinePidRegData; /*55*/
}__attribute__((__packed__)) BLU_LfDataReport_t;/*55bytes total size*/

/////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	float ErrW1;
	float ErrW2;
	float ErrW3;
	float ErrW4;
	float ErrW5;
	float ErrW6;
	float ErrW7;
	float ErrW8;
	float ErrW9;
	float ErrW10;
	float ErrW11;
	float ErrWMax; //Line not detected - beyond path
}BLU_NvM_ErrWeigthSensorData_t; /*Current size 6*4 = 24*/


typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float DerivativeTime;
}BLU_NvM_LinePidRegData_t; /*Current size 6*4 = 24*/

typedef struct
{
	float ExpectedAvSpeed;
	uint8_t BlinkLedState;
	uint8_t TryDetectEndLineMark;
}BLU_NvM_VehCfgData_t; /*Current size 6*4 = 24*/


typedef struct
{
	uint8_t SyncId;
	uint32_t ucTimeStamp;
	uint16_t RingBufferRemainingSize;
	uint16_t RingBufferOverFlowCounter;
	uint16_t TransmisstedMessagesCounter;
	uint16_t RetransmissionCounter;
}BLE_StatisticData_t ;


/*
 *
 * Exported functions prototypes:
 * */

void BLU_Init(void); /*Initialize the Communication module*/
void BLU_Task(void); /*Runnable of BLE communication module (Call as often as possible*/
					/*.. don't blocked contest, very short*/
/**/



/* API  functions: */

/** @brief BLU_isExpectedStateDriving
* @details 
* @return Expected driving by user (follow line or stop)
*/
bool BLU_isExpectedStateDriving(void);


/* brief BLU_ReportSensorData
 * LineEstimator: Dedicated interface for  to transmit sensor data
 */
void BLU_ReportSensorData(BLU_SensorDataReport_t *SensorData);

/* brief BLU_ReportMapData
*LF_Menager: Dedicated interface to create XY map in desktop application
*/
void BLU_ReportMapData(BLU_MapDataReport_t *MapData);

/* brief BLU_RegisterNvMdataUpdateInfoCallBack
* Register call back if you want be informed that Non Volatile data has been updated
* by BLE communication module
* CallBack function will be called always if NvM data will be updated
*/
void BLU_RegisterNvMdataUpdateInfoCallBack(void UpdateInfoCb(void) );

/* brief BLU_RegisterNvMdataUpdateInfoCallBack
* Register call back if, interface dedicated for handler in LF_Menager to cntrl manually 
* the linne follower robot for example during standstill state using a virutal analog cntroller
* in QT desktop application
* CallBack function will be called always when the value of vector in the virtual controller will be changed
*/
void BLU_RegisterManualCntrlRequestCallBack(void ManualCtrlReqCb(float vecV_X, float vecV_Y) );


/* brief BLU_DbgMsgTransmit
* A simple function to send debug message through BLE (to QT Application)
* String length is limited to ~100chars
* Input format is the same as in "printf" function.. 
* To keep good performence i suggest don't sent many debug messages..
*/
void BLU_DbgMsgTransmit(char *DbgString, ...);


#endif //_BLE_ServiceModule_H
