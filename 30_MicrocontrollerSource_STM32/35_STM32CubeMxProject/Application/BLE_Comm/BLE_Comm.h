/**
 * @file BLE_Comm.h
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth low energy module to communicate wireless with Line Follower
 * I used module JDY-08 - if you are interested about information of the modules please find data sheets in internet.
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

#define BLE_TRANSMIT_RING_BUFFER_SIZE     3000

#define BLE_RECEIVE_RING_BUFFER_SIZE     100



/*
 * Defines configuration end
 * */

/*
 * Type defs
 * */

typedef struct
{
	uint8_t BLE_Data[18];
	/*as i know Max size of one BLE message is 20bytes
	 * - |FrameID|SyncID|18bytesData|
	 * */
}BLE_MessageBuffer_t;

/*
 * Type definition of Common Header or message ID for Embedded software and desktop application
 * */
typedef enum
{
    BLE_None = 0,
    BLE_ConfirmationTag,
    BLE_DebugMessage,

    BLE_RobotStart,
    BLE_RobotStop,
	
    BLE_SimulatorStart,
    BLE_TrueBaseLoggingStart,
    BLE_SimuAndTrueDataLoggingStop,

    BLE_CommunicationStatistics,
    BLE_BaseDataReport_part1,
    BLE_BaseDataReport_part2,
    BLE_BaseDataReport_part3,


    BLE_NvM_ErrWeigthSensorDataReq,
    BLE_NvM_ErrWeigthSensorData_part1,
    BLE_NvM_ErrWeigthSensorData_part2,
    BLE_NvM_ErrWeigthSensorData_part3,

    BLE_NvM_LinePidRegDataReq,
    BLE_NvM_LinePidRegData,

    BLE_NvM_VehCfgReq,
    BLE_NvM_VehCfgData,

    BLE_SetNewRobotName,

}BLE_MessageID_t;

typedef enum
{
	BLE_Ok,
	BLE_Error
}BLE_CallStatus_t;



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
	float TravelledDistance;
}BLE_MapDataReport_t; /*Current size 6*4 = 24*/

typedef struct
{
	uint8_t SensorData[12];
	float PosError;
	uint8_t LastLeftLinePosConfidence;
	uint8_t LastRightLinePosConfidence;
}BLE_SensorDataReport_t;  /*Current size= 12+4+1+1 = 18*/

typedef struct
{
    float LinePidRegVal;
}BLE_PidRegData_t; /*Current size= 4*/

typedef struct
{
    uint8_t SyncId;
    uint32_t ucTimeStamp; //5
    BLE_MapDataReport_t CurrMapData;
    BLE_SensorDataReport_t CurrSensorData;
    BLE_PidRegData_t LinePidRegData;
}BLE_LfDataReport_t; /*51bytes total size + 3*2 = 6|" Max 54bytes + 1!! difficult to explain why(+1) (left 4bytes)   -- stil */
					/*However data are aligned to 4bytes... i didn't find time to solve it ;)
					  I know how to do it (very simple - packed structures but i didn't need it ;)
					  it coused that LinePidRegData is on position 16 not 14*/

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
}BLE_NvM_ErrWeigthSensorData_t; /*Current size 6*4 = 24*/


typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float DerivativeTime;
}BLE_NvM_LinePidRegData_t; /*Current size 6*4 = 24*/


typedef struct
{
	float ExpectedAvSpeed;
	uint8_t BlinkLedState;
	uint8_t TryDetectEndLineMark;
}BLE_NvM_VehCfgData_t; /*Current size 6*4 = 24*/




/////////////////////////////////////////////////////////////////////////////////

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

void BLE_Init(void); /*Initialize the Communication module*/
void BLE_Task(void); /*Runnable of BLE communication module (Call as often as possible*/
					/*.. don't blocked contest, very short*/
/**/



/* API  functions: */

/** @brief BLE_isExpectedStateDriving
* @details 
* @return Expected driving by user (follow line or stop)
*/
bool BLE_isExpectedStateDriving(void);


/* brief BLE_ReportSensorData
 * LineEstimator: Dedicated interface for  to transmit sensor data
 */
void BLE_ReportSensorData(BLE_SensorDataReport_t *SensorData);

/* brief BLE_ReportMapData
*LF_Menager: Dedicated interface to create XY map in desktop application
*/
void BLE_ReportMapData(BLE_MapDataReport_t *MapData);

/* brief BLE_RegisterNvMdataUpdateInfoCallBack
* Register call back if you want be informed that Non Volatile data has been updated
* by BLE communication module
* CallBack function will be called always if NvM data will be updated
*/
void BLE_RegisterNvMdataUpdateInfoCallBack(void UpdateInfoCb(void) );

/* brief BLE_DbgMsgTransmit
* A simple function to send debug message through BLE (to QT Application)
* String length is limited to ~255chars
* However please bear in mind that single BLE message size == 20
* Input format is the same as in "printf" function.. 
* To keep good performence i suggest don't sent many debug messages..
*/
void BLE_DbgMsgTransmit(char *DbgString, ...);


#endif //_BLE_ServiceModule_H
