/**
 * @file BLE_Comm.c
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth low energy module to communicate wireless with Line Follower
 * I used module JDY-08 - if you are interested about the modules please find data sheets in internet.
 */


#include "BLE_Comm.h"

#include "stdbool.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "dma.h" /*DMA driver part of STM32 HAL Library*/
#include "usart.h"  /*usart driver part of STM32 HAL Library*/
#include "stddef.h"


#define BLE_SINGLE_MESSAGE_SIZE  20 /*HARD CODED! please don't change it if you aren't rly aware what you are doing*/
									/*Please check BLE specifitaion - max message size is 20bytes*/

#define BLE_DATA_FIELD_SIZE 18

#define BLE_STATISTICS_PERIOD 5000

/**!
 * \brief BleRingBufferStatus_t
 * \details ---
 * */
typedef enum
{
	RB_OK       = 0,
	RB_ERROR	= 1
} BleRingBufferStatus_t;

/**!
 * \brief BleRingBuffer_t
 * \details ---
 * */
typedef struct
{
	uint16_t Head; // Pointer to write
	uint16_t Tail; // Pointer to read
	uint8_t  MessageSize[BLE_RING_BUFFER_SIZE]; // Array to store messages size
} BleRingBuffer_t;



/**
 * *******************************************************************************************
 * Static variables declaration and function prototypes
 * *******************************************************************************************
 * */
static bool LogDroppedFlag =false;
static uint16_t RetransmissionCounter = 0U;
static uint16_t TransmisstedMessagesCounter = 0;

static uint16_t volatile UartBusyCounter =0u;

static BleRingBuffer_t BleMainRingBuffer;
static uint8_t BleMessagesTab[BLE_RING_BUFFER_SIZE][BLE_SINGLE_MESSAGE_SIZE];

/*
 *********************************************************************************************
 * Static function declaration section
 ********************************************************************************************
 */
static BleRingBufferStatus_t BLE_RB_Read(BleRingBuffer_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer);
static BleRingBufferStatus_t BLE_RB_Write(BleRingBuffer_t *Buf,uint8_t *BleLogData, uint8_t MessageSize);


/*TODO: To remove!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/


#define MaxProbeNumberr 300
#define DataBufferSize 20

typedef struct
{
	float X[MaxProbeNumberr];
	float Y[MaxProbeNumberr];
	uint16_t T[MaxProbeNumberr];
}PositionOnTrackL_t;

float LeftWheelSpeedSimulator_f = 7.34F;
float RightWheelSpeedSimulator_f = 3.22F;
uint8_t SyncID = 0;


PositionOnTrackL_t  PositionOnTrackL;

static uint32_t ExtraShifter = 0u;

uint8_t *CreateEncoderModuleMessage(void);
void Create_XY_FakeMap(void);
/*TODO: To remove!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/


/*!
 ************************************************************************************************
 * \brief BLE_RB_Read Function to read data from ring buffer
 * \details --
 * \param RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param out MessageSize - size of the "BleLogData" (return value)
 * \param out MessagePointer - pointer to the message stored in RingBuffer (return value)
 *
 * */
static BleRingBufferStatus_t BLE_RB_Read(BleRingBuffer_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer)
{
	// Check if Tail hit Head
	if(Buf->Head == Buf->Tail)
	{
		// If yes - there is nothing to read
		return RB_ERROR;
	}

	// Write current value from buffer to pointer from argument
	*MessageSize = Buf->MessageSize[Buf->Tail];
	*MessagePointer = &BleMessagesTab[Buf->Tail][0];

	// Calculate new Tail pointer
	Buf->Tail = (Buf->Tail + 1) % BLE_RING_BUFFER_SIZE;

	// Everything is ok - return OK status
	return RB_OK;
}


/*!
 ************************************************************************************************
 * \brief BLE_RB_Write
 * \details --
 * \param in RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param in BleLogData - pointer to the data stored in RingBuffer
 * \param in MessageSize - size of the "BleLogData"
 ************************************************************************************************/
static BleRingBufferStatus_t BLE_RB_Write(BleRingBuffer_t *Buf,uint8_t *DataToSend, uint8_t MessageSize)
{
	uint16_t ActualWriteIndex;

	// Calculate new Head pointer value
	uint16_t HeadTmp = (Buf->Head + 1) % BLE_RING_BUFFER_SIZE;

	// Check if there is one free space ahead the Head buffer
	if(HeadTmp == Buf->Tail)
	{
		// There is no space in the buffer - return an error
		return RB_ERROR;
	}

	ActualWriteIndex = Buf->Head;
	Buf->Head = HeadTmp;
	// Store a value into the buffer
	Buf->MessageSize[ActualWriteIndex] = MessageSize;

	/*Copy the values to new buffer*/
	for(int i=0; i<MessageSize; i++)
	{
		BleMessagesTab[ActualWriteIndex][i] = DataToSend[i];
	}
	// Everything is ok - return OK status
	return RB_OK;
}






static void Statistics_CreateAndTransmitCommunicationStatistics(void)
{
	static uint8_t RingBufferStatisticsHolder[20] = {0};

	static uint16_t MainRingBufferOverFlowCounter = 0U;
	uint16_t RingBufferRemaninzingSpace = 0U;

	for(int i =0; i<20; i++)
	{
		RingBufferStatisticsHolder[i] = 0U; /*Sanitize the buffer*/
	}

	if(true == LogDroppedFlag)
	{
		LogDroppedFlag = false;
		MainRingBufferOverFlowCounter++;
	}

	RingBufferStatisticsHolder[0] = BLE_CommunicationStatistics;
	RingBufferStatisticsHolder[1] = 0U; /*SyncId*/

	RingBufferRemaninzingSpace = BLE_RING_BUFFER_SIZE -(BleMainRingBuffer.Tail -BleMainRingBuffer.Head);

	RingBufferStatisticsHolder[2] = ((uint8_t *)&RingBufferRemaninzingSpace)[1];
	RingBufferStatisticsHolder[3] = ((uint8_t *)&RingBufferRemaninzingSpace)[0];

	RingBufferStatisticsHolder[5] = ((uint8_t *)&MainRingBufferOverFlowCounter)[1];
	RingBufferStatisticsHolder[6] = ((uint8_t *)&MainRingBufferOverFlowCounter)[0];


	RingBufferStatisticsHolder[9]  =  ((uint8_t *)&RetransmissionCounter)[1];
	RingBufferStatisticsHolder[10] =  ((uint8_t *)&RetransmissionCounter)[0];

	RingBufferStatisticsHolder[11] =  ((uint8_t *)&TransmisstedMessagesCounter)[1];
	RingBufferStatisticsHolder[12] =  ((uint8_t *)&TransmisstedMessagesCounter)[0];


	RingBufferStatisticsHolder[18] = '\n';
	RingBufferStatisticsHolder[19] = '\r';

	HAL_UART_Transmit_DMA(&huart2, RingBufferStatisticsHolder, BLE_SINGLE_MESSAGE_SIZE);
}


/*
 ****************************************************************************************************
 * exported functions declarations section START
 * exported functions description is added in header file
 *****************************************************************************************************
 */

void TemporaryFakeProducer(void)
{
	static uint16_t ProbeIterator = 0u;
	static uint8_t SyncIdIter = 1;

	if(ProbeIterator == MaxProbeNumberr)
	{
		Create_XY_FakeMap();
		ProbeIterator = 0U;
	}
	ProbeIterator++;

	BLE_MapData_t CurrMapData = {0};

	CurrMapData.SyncId = SyncIdIter;
	SyncIdIter++;
	CurrMapData.ucTimeStamp = HAL_GetTick();
	CurrMapData.PosX = PositionOnTrackL.X[ProbeIterator];
	CurrMapData.PosY = PositionOnTrackL.Y[ProbeIterator];
	CurrMapData.WhLftSp = 7.0F;
	CurrMapData.WhRhtSp = 9.0F;
	CurrMapData.YawRate = 5.0F;
	BLE_MapDataWrite(&CurrMapData);
}

void BLE_Init(void)
{
	Create_XY_FakeMap();
}

void BLE_Task(void)
{

	static uint32_t BLE_DataTransmitWindow = 0;
	static bool TransmitWindowActiv = 0;

	static uint32_t BLE_StatisticTimer = 0;
	static uint32_t TempFakeProducerTimer = 0;

	if(HAL_GetTick() - TempFakeProducerTimer >= 20)
	{
		TempFakeProducerTimer = HAL_GetTick();
		for(int i =0; i<2; i++)
		{
			TemporaryFakeProducer();
		}
	}

/*Fajnie wyszlo na 220 wobec 50*/
	if(HAL_GetTick() - BLE_DataTransmitWindow > 250)
	{
		BLE_DataTransmitWindow = HAL_GetTick();
		TransmitWindowActiv = true;
	}

	if(HAL_GetTick() - BLE_DataTransmitWindow > 20)
	{
		TransmitWindowActiv = false;
	}

	if( HAL_UART_STATE_READY == huart2.gState && (true == TransmitWindowActiv))
	{
		if(HAL_GetTick() - BLE_StatisticTimer > BLE_STATISTICS_PERIOD)
		{
				BLE_StatisticTimer = HAL_GetTick();
				/*Transmit the statistics with higher prio than normal messages*/
				TransmisstedMessagesCounter++;
				Statistics_CreateAndTransmitCommunicationStatistics();
		}
		else
		{
				uint8_t *MessageToTransmit_p = NULL;
				uint8_t MessageToSize_p = 0U;
				if(BLE_RB_Read(&BleMainRingBuffer, &MessageToSize_p, &MessageToTransmit_p) == RB_OK)
				{
					TransmisstedMessagesCounter++;
					HAL_UART_Transmit_DMA(&huart2, MessageToTransmit_p, MessageToSize_p);
				}
		}
	}
	else
	{
		UartBusyCounter++;
	}
}


BLE_CallStatus_t BLE_MessageWrite(BLE_MessageID_t MessageID,uint8_t SyncId,BLE_MessageBuffer_t *MessageData)
{
	uint8_t _tempDataTab[20];
	BLE_CallStatus_t retval = BLE_Ok;

	_tempDataTab[0] = MessageID;
	_tempDataTab[1] = SyncId;
	for(int i=0; i<BLE_DATA_FIELD_SIZE; i++)
	{
		_tempDataTab[2+i] = MessageData->BLE_Data[i];
	}

	if(BLE_RB_Write(&BleMainRingBuffer, _tempDataTab, BLE_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	return retval;
}

BLE_CallStatus_t BLE_MapDataWrite(BLE_MapData_t *MapData)
{
	uint8_t BaseMapDataBuffHelper[20] = {0};
	uint8_t ExtraMapDataBuffHelper[20]= {0};
	BLE_CallStatus_t retval = BLE_Ok;

	BaseMapDataBuffHelper[0] = BLE_BaseMapData;
	BaseMapDataBuffHelper[1] = MapData->SyncId;

	ExtraMapDataBuffHelper[0] = BLE_ExtraMapData;
	ExtraMapDataBuffHelper[1] = MapData->SyncId;


	BaseMapDataBuffHelper[18] = '\n';
	BaseMapDataBuffHelper[19] = '\r';

	ExtraMapDataBuffHelper[18] = '\n';
	ExtraMapDataBuffHelper[19] = '\r';


	memcpy(&BaseMapDataBuffHelper[2], &MapData->ucTimeStamp,16 );
					/*16 - frist 4fields of data 4*4 YawRate,WhRhtSp,WhLftSp,ucTime*/

	memcpy(&ExtraMapDataBuffHelper[2], &MapData->PosX,12 ); 
						/*12 - 3fields of data 3*4 PosX,PosY,TravelledDistance*/

	if(BLE_RB_Write(&BleMainRingBuffer, BaseMapDataBuffHelper, BLE_SINGLE_MESSAGE_SIZE) != RB_OK
		|| BLE_RB_Write(&BleMainRingBuffer, ExtraMapDataBuffHelper, BLE_SINGLE_MESSAGE_SIZE) != RB_OK
	)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	return retval;
}


BLE_CallStatus_t BLE_SensorDataWrite(BLE_SensorData_t *SensorData)
{
	uint8_t BaseSensorDataWrite[20] = {0};
	uint8_t ExtraSensorData[20] = {0};

	BLE_CallStatus_t retval = BLE_Ok;

	BaseSensorDataWrite[0] = BLE_BaseSensorData;
	BaseSensorDataWrite[1] = SensorData->SyncId;

	ExtraSensorData[0] = BLE_ExtraSensorData;
	ExtraSensorData[1] = SensorData->SyncId;

	memcpy(&BaseSensorDataWrite[2], &SensorData->ucTimeStamp,16 );
					/*16 - 12bytes of sensor data and 4 bytes for ucTimeStamp*/

	memcpy(&ExtraSensorData[2], &SensorData->PosError,6 );
						/*6 - 4 for float PosError and 2 for LastRightLinePosConfidence and LastLeftLinePosConfidence*/

	if(BLE_RB_Write(&BleMainRingBuffer, BaseSensorDataWrite, BLE_SINGLE_MESSAGE_SIZE) != RB_OK
		|| BLE_RB_Write(&BleMainRingBuffer, ExtraSensorData, BLE_SINGLE_MESSAGE_SIZE) != RB_OK
	)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	return retval;
}


void Create_XY_FakeMap(void)
{
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i]=    HAL_GetTick();
		PositionOnTrackL.X[i]= i + ExtraShifter;
		PositionOnTrackL.Y[i] = 0 + ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i+100]= HAL_GetTick();
		PositionOnTrackL.X[i+100]= 100 + ExtraShifter;
		PositionOnTrackL.Y[i+100] = i + ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i+200]= HAL_GetTick();
		PositionOnTrackL.X[i+200]= 100 - i + ExtraShifter;
		PositionOnTrackL.Y[i+200] =100 + ExtraShifter;
	}

	ExtraShifter = ExtraShifter + 5;
}
















