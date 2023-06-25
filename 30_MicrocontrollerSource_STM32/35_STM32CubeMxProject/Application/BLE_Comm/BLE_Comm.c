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
#include "math.h"


#define BLE_MIN_SINGLE_MESSAGE_SIZE  20 /*HARD CODED! please don't change it if you aren't rly aware what you are doing*/
									/*Please check BLE specifitaion - max message size is 20bytes*/

#define BLE_MAX_SINGLE_MESSAGE_SIZE  (3 * BLE_MIN_SINGLE_MESSAGE_SIZE)  /*HARD CODED! please don't change it if you aren't rly aware what you are doing*/
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
static uint8_t BleMessagesTab[BLE_RING_BUFFER_SIZE][BLE_MAX_SINGLE_MESSAGE_SIZE];

static BLE_LfDataReport_t NewestLfDataReport = {0};

static bool FakeProducerStateActiv = false;


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


static uint8_t BLE_ReceiveBuffer[50];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// Check if UART2 trigger the Callback
	if(huart->Instance == USART2)
	{

		if(BLE_ReceiveBuffer[0] == BLE_SuspendFakeProducer)
		{
			FakeProducerStateActiv = false;
		}

		if(BLE_ReceiveBuffer[0] == BLE_StartFakeProducer)
		{
			FakeProducerStateActiv = true;
		}
		// Start listening again

 		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, BLE_ReceiveBuffer, 20);
//		BLE_RxEventCallback(Size); //Application/Src/BLE_ServiceModule.c
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}
}



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


static BLE_CallStatus_t BLE_TransmitLfDataReport(void)
{
	uint8_t DataBuffer[3][20] = {0};
	static uint8_t _SyncID = 0;

	BLE_CallStatus_t retval = BLE_Ok;

	NewestLfDataReport.SyncId = _SyncID;
	NewestLfDataReport.ucTimeStamp = HAL_GetTick();

	DataBuffer[0][0] = BLE_BaseDataReport_part1;
	DataBuffer[0][1] = NewestLfDataReport.SyncId;
	DataBuffer[1][0] = BLE_BaseDataReport_part2;
	DataBuffer[1][1] = NewestLfDataReport.SyncId;
	DataBuffer[2][0] = BLE_BaseDataReport_part3;
	DataBuffer[2][1] = NewestLfDataReport.SyncId;

	memcpy(&DataBuffer[0][2], &NewestLfDataReport.ucTimeStamp,16); /*Timestamp, WhLftSp,WhRhtSp,YawRate*/
	memcpy(&DataBuffer[1][2], &NewestLfDataReport.CurrMapData.PosX,18); /*PosX, PosY, TravelledDistance, 6BytesOfSensorData*/
	memcpy(&DataBuffer[2][2], &NewestLfDataReport.CurrSensorData.SensorData[6],18); /*6bytes of sensorData,PosError, 2* LinePosConfidence*/

	if(BLE_RB_Write(&BleMainRingBuffer, (uint8_t *)DataBuffer, BLE_MAX_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	_SyncID++;
	return retval;
}



static void Statistics_CreateAndTransmitCommunicationStatistics(void)
{
	static uint8_t SyncIdStat = 0U;

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
	RingBufferStatisticsHolder[1] = SyncIdStat; /*SyncId*/

	uint32_t ucHelperTime = HAL_GetTick();
	RingBufferStatisticsHolder[2] = ((uint8_t *)&ucHelperTime)[3];
	RingBufferStatisticsHolder[3] = ((uint8_t *)&ucHelperTime)[2];
	RingBufferStatisticsHolder[4] = ((uint8_t *)&ucHelperTime)[1];
	RingBufferStatisticsHolder[5] = ((uint8_t *)&ucHelperTime)[0];

	RingBufferRemaninzingSpace = BLE_RING_BUFFER_SIZE - (BleMainRingBuffer.Head - BleMainRingBuffer.Tail);

	RingBufferStatisticsHolder[6] = ((uint8_t *)&RingBufferRemaninzingSpace)[0];
	RingBufferStatisticsHolder[7] = ((uint8_t *)&RingBufferRemaninzingSpace)[1];

	RingBufferStatisticsHolder[8] = ((uint8_t *)&MainRingBufferOverFlowCounter)[0];
	RingBufferStatisticsHolder[9] = ((uint8_t *)&MainRingBufferOverFlowCounter)[1];

	RingBufferStatisticsHolder[10] =  ((uint8_t *)&TransmisstedMessagesCounter)[0];
	RingBufferStatisticsHolder[11] =  ((uint8_t *)&TransmisstedMessagesCounter)[1];

	RingBufferStatisticsHolder[12]  =  ((uint8_t *)&RetransmissionCounter)[0];
	RingBufferStatisticsHolder[13] =  ((uint8_t *)&RetransmissionCounter)[1];

	HAL_UART_Transmit_DMA(&huart2, RingBufferStatisticsHolder, BLE_MIN_SINGLE_MESSAGE_SIZE);

	SyncIdStat++;
}


/*
 ****************************************************************************************************
 * exported functions declarations section START
 * exported functions description is added in header file
 *****************************************************************************************************
 */

void Create_XY_FakeMap(void)
{
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i]=    HAL_GetTick();
		PositionOnTrackL.X[i]=  (float)i + (float)ExtraShifter;
		PositionOnTrackL.Y[i] = 0.0F + (float)ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i+100]= HAL_GetTick();
		PositionOnTrackL.X[i+100]= 100.0F + (float)ExtraShifter;
		PositionOnTrackL.Y[i+100] = (float)i  + (float)ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i+200]= HAL_GetTick();
		PositionOnTrackL.X[i+200]= 100.0F - ((float)i*1.0F) + (float)ExtraShifter;
		PositionOnTrackL.Y[i+200] =100.0F     + (float)ExtraShifter;
	}

	ExtraShifter = ExtraShifter + 5;
}

void TemporaryFakeProducer(void)
{
	static uint16_t ProbeIterator = 0u;
	static uint8_t SyncIdIter = 1;
	static float WaveHelper = 0.0F;

	if(ProbeIterator == MaxProbeNumberr)
	{
		Create_XY_FakeMap();
		ProbeIterator = 1U;
	}

	NewestLfDataReport.ucTimeStamp = HAL_GetTick();

	NewestLfDataReport.CurrMapData.PosX = PositionOnTrackL.X[ProbeIterator];
	NewestLfDataReport.CurrMapData.PosY = PositionOnTrackL.Y[ProbeIterator];

	NewestLfDataReport.CurrMapData.WhLftSp = (2 * sin( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 100 * WaveHelper));
	NewestLfDataReport.CurrMapData.WhRhtSp = (2 * sin( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 50 * WaveHelper));

	NewestLfDataReport.CurrMapData.YawRate = (2 * sin( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 50 * WaveHelper));


	NewestLfDataReport.CurrSensorData.SensorData[0] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[1] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[2] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[3] = 63;
	NewestLfDataReport.CurrSensorData.SensorData[4] = 64;
	NewestLfDataReport.CurrSensorData.SensorData[5] = (uint8_t)(65 + SyncIdIter);
	NewestLfDataReport.CurrSensorData.SensorData[6] = (uint8_t)(66 + SyncIdIter);
	NewestLfDataReport.CurrSensorData.SensorData[7] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[8] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[9] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[10] = 40;
	NewestLfDataReport.CurrSensorData.SensorData[11] = 40;

	NewestLfDataReport.CurrSensorData.PosError = 1 * sin( 2 * M_PI * WaveHelper);

	BLE_TransmitLfDataReport();

	WaveHelper = WaveHelper + 0.05;
	ProbeIterator++;
	SyncIdIter++;
}

void BLE_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, BLE_ReceiveBuffer, 20);
	Create_XY_FakeMap();
}

void BLE_Task(void)
{

	static uint32_t BLE_DataTransmitWindow = 0;
	static bool TransmitWindowActiv = 0;

	static uint32_t BLE_StatisticTimer = 0;
	static uint32_t TempFakeProducerTimer = 0;

	if(HAL_GetTick() - TempFakeProducerTimer >= 20 && (true == FakeProducerStateActiv) )
	{
		TempFakeProducerTimer = HAL_GetTick();
		for(int i =0; i<1; i++)
		{
			TemporaryFakeProducer();
		}
	}


	TransmitWindowActiv = true;
	(void)BLE_DataTransmitWindow;
/*Fajnie wyszlo na 200 wobec 60*/
//	if(HAL_GetTick() - BLE_DataTransmitWindow > 300)
//	{
//		BLE_DataTransmitWindow = HAL_GetTick();
//		TransmitWindowActiv = true;
//	}
//
//	if(HAL_GetTick() - BLE_DataTransmitWindow > 40)
//	{
//		TransmitWindowActiv = false;
//	}

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

	if(BLE_RB_Write(&BleMainRingBuffer, _tempDataTab, BLE_MIN_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	return retval;
}


void BLE_ReportMapData(BLE_MapDataReport_t *MapData)
{
	NewestLfDataReport.CurrMapData = *MapData;
}

void BLE_ReportSensorData(BLE_SensorDataReport_t *SensorData)
{
	NewestLfDataReport.CurrSensorData = *SensorData;
}



















