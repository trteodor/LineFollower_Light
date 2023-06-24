/**
 * @file BLE_Comm.h
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth low energy module to communicate wireless with Line Follower
 * I used module JDY-08 - if you are interested about the modules please find data sheets in internet.
 */





#ifndef _BLE_ServiceModule_H
#define _BLE_ServiceModule_H

#include "stdint.h"
#include "stdbool.h"


/*
 * Defines configuration start
 * */

#define BLE_RING_BUFFER_SIZE     7000





/*
 * Defines configuration end
 * */

/*
 * Type defs
 * */


/*
 * Type definition of Common Header or message ID for Embedded software and desktop application
 * */
typedef enum
{
	BLE_None = 0,
	BLE_ConfirmationTag,
	BLE_CommunicationStatistics,
	BLE_BaseSensorData, /*FRAME: Line Sensors 1-12 as 8bit, PosError, ConfidenceLine Position Left and Right*/
	BLE_ExtraSensorData,
	BLE_BaseMapData,
	BLE_ExtraMapData,

}BLE_MessageID_t;

typedef enum
{
	BLE_Ok,
	BLE_Error
}BLE_CallStatus_t;


typedef struct
{
	uint8_t BLE_Data[18];
	/*as i know Max size of one BLE message is 20bytes
	 * - |FrameID|SyncID|18bytesData|
	 * */
}BLE_MessageBuffer_t;

typedef struct
{
	uint8_t SyncId;
	uint32_t ucTimeStamp;
	float WhLftSp;
	float WhRhtSp;
	float YawRate;
	float PosX;
	float PosY;
	float TravelledDistance;
}BLE_MapData_t;

typedef struct
{
	uint8_t SyncId;
	uint32_t ucTimeStamp;
	uint8_t SensorData[12];
	float PosError;
	uint8_t LastLeftLinePosConfidence;
	uint8_t LastRightLinePosConfidence;
}BLE_SensorData_t;



/*
 *
 * Exported functions prototypes:
 * */

void BLE_init(void); /*Initialize the Communication module*/
void BLE_Task(void); /*Runnable of BLE communication module (Call as often as possible*/
					/*.. don't blocked contest, very short*/
/**/



/* API  functions: */

/* brief BLE_MessageWrite
 * General interface to transmit message - strongly suggest to don't use
 * */
BLE_CallStatus_t BLE_MessageWrite(BLE_MessageID_t MessageID,uint8_t SyncId,BLE_MessageBuffer_t *MessageData);


/* brief BLE_SensorDataWrite
 * LineEstimator: Dedicated interface for  to transmit sensor data
 */
BLE_CallStatus_t BLE_SensorDataWrite(BLE_SensorData_t *SensorData);


/* brief BLE_MapDataWrite
*LF_AppMain: Dedicated interface to create XY map in desktop application
*/
BLE_CallStatus_t BLE_MapDataWrite(BLE_MapData_t *MapData);



#endif //_BLE_ServiceModule_H
