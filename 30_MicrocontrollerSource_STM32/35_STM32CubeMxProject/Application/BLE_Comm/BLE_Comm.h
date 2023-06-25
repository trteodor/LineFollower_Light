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

#define BLE_RING_BUFFER_SIZE     3000





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
	BLE_CommunicationStatistics,
	BLE_BaseDataReport_part1, /*Divide most necessary data into 1 compressed buffor to effienctly communicate*/
	BLE_BaseDataReport_part2,
	BLE_BaseDataReport_part3,
	BLE_SuspendFakeProducer,
	BLE_StartFakeProducer,
}BLE_MessageID_t;

typedef enum
{
	BLE_Ok,
	BLE_Error
}BLE_CallStatus_t;



/*
 * Modules should report the newest data then BLE module will transmit it
 * */



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
	uint8_t SyncId;
	uint32_t ucTimeStamp;
	BLE_MapDataReport_t CurrMapData;
	BLE_SensorDataReport_t CurrSensorData;
}BLE_LfDataReport_t ; /*47bytes total size + 3*2 = 6*/


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

void BLE_init(void); /*Initialize the Communication module*/
void BLE_Task(void); /*Runnable of BLE communication module (Call as often as possible*/
					/*.. don't blocked contest, very short*/
/**/



/* API  functions: */

/* brief BLE_MessageWrite
 * General interface to transmit message - strongly suggest to don't use
 * */
BLE_CallStatus_t BLE_MessageWrite(BLE_MessageID_t MessageID,uint8_t SyncId,BLE_MessageBuffer_t *MessageData);


/* brief BLE_ReportSensorData
 * LineEstimator: Dedicated interface for  to transmit sensor data
 */
void BLE_ReportSensorData(BLE_SensorDataReport_t *SensorData);

/* brief BLE_ReportMapData
*LF_AppMain: Dedicated interface to create XY map in desktop application
*/
void BLE_ReportMapData(BLE_MapDataReport_t *MapData);



#endif //_BLE_ServiceModule_H
