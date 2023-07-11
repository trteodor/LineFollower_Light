/**
 * @file BLE_Comm.c
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth low energy module to communicate wireless with Line Follower
 * I used module JDY-08 - if you are interested about information of the modules please find data sheets in internet.
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

#include "EEmu.h" /*EEPROM emulation header*/
#include "EEmuConfig.h"

#define SIMULATOR_PROBES_COUNT 300

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
 * \brief BleRingBufferTransmit_t
 * \details ---
 * */
typedef struct
{
	uint16_t Head; // Pointer to write
	uint16_t Tail; // Pointer to read
	uint8_t  MessageSize[BLE_TRANSMIT_RING_BUFFER_SIZE]; // Array to store messages size
} BleRingBufferTransmit_t;

/**!
 * \brief BleRingBufferReceive_t
 * \details ---
 * */
typedef struct
{

	uint16_t Head; // Pointer to write
	uint16_t Tail; // Pointer to read
	uint8_t  MessageSize[BLE_RECEIVE_RING_BUFFER_SIZE]; // Array to store messages size
	bool ReadyToRead[BLE_RECEIVE_RING_BUFFER_SIZE];
} BleRingBufferReceive_t;

typedef struct
{
	float X[SIMULATOR_PROBES_COUNT];
	float Y[SIMULATOR_PROBES_COUNT];
	uint16_t T[SIMULATOR_PROBES_COUNT];
}Sim_PositionOnTruck_t;



/**
 * *******************************************************************************************
 * Static variables declaration and function prototypes
 * *******************************************************************************************
 * */

Sim_PositionOnTruck_t  SimFakeXY_MapDat;

static bool LogDroppedFlag =false;
static uint16_t RetransmissionCounter = 0U;
static uint16_t TransmisstedMessagesCounter = 0;

static uint16_t volatile UartBusyCounter =0u;

static BleRingBufferTransmit_t BleMainTransmitRingBuffer;
static uint8_t BleMainTransmitMessagesTab[BLE_TRANSMIT_RING_BUFFER_SIZE][BLE_MAX_SINGLE_MESSAGE_SIZE];

static BleRingBufferReceive_t BleMainReceiveRingBuffer;
static uint8_t BleMainReceiveMessagesTab[BLE_RECEIVE_RING_BUFFER_SIZE][BLE_MAX_SINGLE_MESSAGE_SIZE];


static BLE_LfDataReport_t NewestLfDataReport = {0};

static bool SimulatorBaseDataStateActiv = false;



/*
 *********************************************************************************************
 * Static function declaration section
 ********************************************************************************************
 */

static void Sim_Create_XY_FakeMap(void);

static BleRingBufferStatus_t RB_Transmit_Read(BleRingBufferTransmit_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer);
static BleRingBufferStatus_t RB_Transmit_Write(BleRingBufferTransmit_t *Buf,uint8_t *DataToWrite, uint8_t MessageSize);


static BleRingBufferStatus_t RB_Receive_GetNextMessageAddress(BleRingBufferReceive_t *Buf, uint8_t **WriteAddress);
static BleRingBufferStatus_t RB_Receive_Read(BleRingBufferReceive_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer);


static BLE_CallStatus_t TransmitErrorWeigthData(void);

static BLE_CallStatus_t MessageWrite(BLE_MessageID_t MessageID,uint8_t SyncId,BLE_MessageBuffer_t *MessageData);





void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if(huart->Instance == USART2)
	{
		uint8_t *MessageReceiveBufferAddress;

		/*Start listen again as fast as possible*/
		RB_Receive_GetNextMessageAddress(&BleMainReceiveRingBuffer,&MessageReceiveBufferAddress);
		// Start listening again
 		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, MessageReceiveBufferAddress, BLE_MIN_SINGLE_MESSAGE_SIZE);

		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

	}
}


/*!
 ************************************************************************************************
 * \brief RB_Transmit_Read Function to read data from ring buffer
 * \details --
 * \param RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param out MessageSize - size of the "BleLogData" (return value)
 * \param out MessagePointer - pointer to the message stored in RingBuffer (return value)
 *
 * */
static BleRingBufferStatus_t RB_Transmit_Read(BleRingBufferTransmit_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer)
{
	// Check if Tail hit Head
	if(Buf->Head == Buf->Tail)
	{
		// If yes - there is nothing to read
		return RB_ERROR;
	}

	// Write current value from buffer to pointer from argument
	*MessageSize = Buf->MessageSize[Buf->Tail];
	*MessagePointer = &BleMainTransmitMessagesTab[Buf->Tail][0];

	// Calculate new Tail pointer
	Buf->Tail = (Buf->Tail + 1) % BLE_TRANSMIT_RING_BUFFER_SIZE;

	// Everything is ok - return OK status
	return RB_OK;
}


/*!
 ************************************************************************************************
 * \brief RB_Transmit_Write
 * \details --
 * \param in RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param in BleLogData - pointer to the data stored in RingBuffer
 * \param in MessageSize - size of the "BleLogData"
 ************************************************************************************************/
static BleRingBufferStatus_t RB_Transmit_Write(BleRingBufferTransmit_t *Buf,uint8_t *DataToWrite, uint8_t MessageSize)
{
	uint16_t ActualWriteIndex;

	// Calculate new Head pointer value
	uint16_t HeadTmp = (Buf->Head + 1) % BLE_TRANSMIT_RING_BUFFER_SIZE;

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
		BleMainTransmitMessagesTab[ActualWriteIndex][i] = DataToWrite[i];
	}
	// Everything is ok - return OK status
	return RB_OK;
}

/*!
 ************************************************************************************************
 * \brief BLE_RB_GetWriteIndexAndMarkAsWritten
 * \details Function used to work with DMA - direct write to ring buffer by DMA
 * \param in WriteIndex - Current Write index in Ring Buffer where the 
 *                          value should be transmitted for example by DMA
 ************************************************************************************************/
static BleRingBufferStatus_t RB_Receive_GetNextMessageAddress(BleRingBufferReceive_t *Buf, uint8_t **WriteAddress)
{
	static uint8_t DefaultBlindBuffer[20];



	/*Mark previous message as ready to read*/
	Buf->ReadyToRead[Buf->Head] = true;

	// Calculate new Head pointer value
	uint16_t HeadTmp = (Buf->Head + 1) % BLE_RECEIVE_RING_BUFFER_SIZE;

	// Check if there is one free space ahead the Head buffer
	if(HeadTmp == Buf->Tail)
	{
		/*Even if buffer is full data must be received somewhere to don't crush application/ dma*/
		*WriteAddress = DefaultBlindBuffer;
		// There is no space in the buffer - return an error
		return RB_ERROR;
	}

	Buf->ReadyToRead[HeadTmp] = false;
	Buf->MessageSize[HeadTmp] = BLE_MIN_SINGLE_MESSAGE_SIZE;
	Buf->Head = HeadTmp;

	*WriteAddress = &BleMainReceiveMessagesTab[HeadTmp][0];

	// Everything is ok - return OK status
	return RB_OK;
}


/*!
 ************************************************************************************************
 * \brief RB_Transmit_Read Function to read data from ring buffer
 * \details --
 * \param RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param out MessageSize - size of the "BleLogData" (return value)
 * \param out MessagePointer - pointer to the message stored in RingBuffer (return value)
 *
 * */
static BleRingBufferStatus_t RB_Receive_Read(BleRingBufferReceive_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer)
{

	if(Buf->ReadyToRead[Buf->Tail] == false)
	{
		/*Any message in ring buffer isn't ready to read*/
		return RB_ERROR;
	}
	/*Mark again as not ready to read*/
	Buf->ReadyToRead[Buf->Tail] = false;

	// Check if Tail hit Head
	if(Buf->Head == Buf->Tail)
	{
		// If yes - there is nothing to read
		return RB_ERROR;
	}



	// Write current value from buffer to pointer from argument
	*MessageSize = Buf->MessageSize[Buf->Tail];
	*MessagePointer = &BleMainReceiveMessagesTab[Buf->Tail][0];

	// Calculate new Tail pointer
	Buf->Tail = (Buf->Tail + 1) % BLE_RECEIVE_RING_BUFFER_SIZE;

	// Everything is ok - return OK status
	return RB_OK;
}

static BLE_CallStatus_t TransmitLfBaseDataReport(void)
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


	/*!!!2+18 = 20!!!!*/
	memcpy(&DataBuffer[0][2], &NewestLfDataReport.ucTimeStamp,16); /*2bytes left only*//*Timestamp, WhLftSp,WhRhtSp,YawRate*/
	memcpy(&DataBuffer[1][2], &NewestLfDataReport.CurrMapData.PosX,18); /*PosX, PosY, TravelledDistance, 6BytesOfSensorData*/
	memcpy(&DataBuffer[2][2], &NewestLfDataReport.CurrSensorData.SensorData[6],18); /*6bytes of sensorData,PosError, 2* LinePosConfidence
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 However data are align to 4bytes... i didn't find time to solve it ;)
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 I know how to do it (packed structures but i didn't need it ;) */

	if(RB_Transmit_Write(&BleMainTransmitRingBuffer, (uint8_t *)DataBuffer, BLE_MAX_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	_SyncID++;
	return retval;
}

static BLE_CallStatus_t TransmitErrorWeigthData(void)
{
	uint8_t DataBuffer[3][20] = {0};
	static uint8_t _SyncID = 0;

	BLE_CallStatus_t retval = BLE_Ok;

	float ErrWDataP1,ErrWDataP2,ErrWDataP3,ErrWDataP4;

	/***********************************************************************/
	/*Prepare transmit data of part 1*/
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth1_F32, &ErrWDataP1);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth2_F32, &ErrWDataP2);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth3_F32, &ErrWDataP3);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth4_F32, &ErrWDataP4);

	DataBuffer[0][0] = BLE_NvM_ErrWeigthSensorData_part1;
	DataBuffer[0][1] = _SyncID;
	memcpy(&DataBuffer[0][2],&ErrWDataP1,4);
	memcpy(&DataBuffer[0][6],&ErrWDataP2,4);
	memcpy(&DataBuffer[0][10],&ErrWDataP3,4);
	memcpy(&DataBuffer[0][14],&ErrWDataP4,4);

	/***********************************************************************/
	/*Prepare transmit data of part 2*/
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth5_F32, &ErrWDataP1);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth6_F32, &ErrWDataP2);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth7_F32, &ErrWDataP3);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth8_F32, &ErrWDataP4);

	DataBuffer[1][0] = BLE_NvM_ErrWeigthSensorData_part2;
	DataBuffer[1][1] = _SyncID;
	memcpy(&DataBuffer[1][2],&ErrWDataP1,4);
	memcpy(&DataBuffer[1][6],&ErrWDataP2,4);
	memcpy(&DataBuffer[1][10],&ErrWDataP3,4);
	memcpy(&DataBuffer[1][14],&ErrWDataP4,4);
	/***********************************************************************/
	/*Prepare transmit data of part 3*/
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth9_F32, &ErrWDataP1);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth10_F32, &ErrWDataP2);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigth11_F32, &ErrWDataP3);
	EE_ReadVariableF32(EE_NvmAddr_SenErrWeigthMax_F32, &ErrWDataP4);

	DataBuffer[2][0] = BLE_NvM_ErrWeigthSensorData_part3;
	DataBuffer[2][1] = _SyncID;
	memcpy(&DataBuffer[2][2],&ErrWDataP1,4);
	memcpy(&DataBuffer[2][6],&ErrWDataP2,4);
	memcpy(&DataBuffer[2][10],&ErrWDataP3,4);
	memcpy(&DataBuffer[2][14],&ErrWDataP4,4);
	/***********************************************************************/

	if(RB_Transmit_Write(&BleMainTransmitRingBuffer, (uint8_t *)DataBuffer, BLE_MAX_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	_SyncID++;
	return retval;
}

static BLE_CallStatus_t TransmitPidData(void)
{
	uint8_t DataBuffer[20] = {0};
	static uint8_t _SyncID = 0;

	BLE_CallStatus_t retval = BLE_Ok;

	float PidKp,PidKi,PidKd,ProbeT;

	/***********************************************************************/
	EE_ReadVariableF32(EE_NvmAddr_PidKp_F32, &PidKp);
	EE_ReadVariableF32(EE_NvmAddr_PidKi_F32, &PidKi);
	EE_ReadVariableF32(EE_NvmAddr_PidKd_F32, &PidKd);
	EE_ReadVariableF32(EE_NvmAddr_ProbeTime_U32, &ProbeT);

	DataBuffer[0] = BLE_NvM_PidRegData;
	DataBuffer[1] = _SyncID;
	memcpy(&DataBuffer[2],&PidKp,4);
	memcpy(&DataBuffer[6],&PidKi,4);
	memcpy(&DataBuffer[10],&PidKd,4);
	memcpy(&DataBuffer[14],&ProbeT,4);

	if(RB_Transmit_Write(&BleMainTransmitRingBuffer, (uint8_t *)DataBuffer, BLE_MIN_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	_SyncID++;
	return retval;
}

static BLE_CallStatus_t TransmitVehCfgData(void)
{
	uint8_t DataBuffer[20] = {0};
	static uint8_t _SyncID = 0;

	BLE_CallStatus_t retval = BLE_Ok;

	float BaseMSpd;
	uint32_t BlinkLedSt,TryDetEndLineMFlag;

	/***********************************************************************/
	EE_ReadVariableF32(EE_NvmAddr_BaseMotorSpdValue_U32, &BaseMSpd);
	EE_ReadVariableU32(EE_NvmAddr_BlinkLadeState_U32, &BlinkLedSt);
	EE_ReadVariableU32(EE_NvmAddr_TryDetectEndLine_U32, &TryDetEndLineMFlag);


	DataBuffer[0] = BLE_NvM_VehCfgData;
	DataBuffer[1] = _SyncID;
	memcpy(&DataBuffer[2],&BaseMSpd,4);
	memcpy(&DataBuffer[6],&BlinkLedSt,4);
	memcpy(&DataBuffer[10],&TryDetEndLineMFlag,4);

	if(RB_Transmit_Write(&BleMainTransmitRingBuffer, (uint8_t *)DataBuffer, BLE_MIN_SINGLE_MESSAGE_SIZE) != RB_OK)
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
	RingBufferStatisticsHolder[2] = ((uint8_t *)&ucHelperTime)[0];
	RingBufferStatisticsHolder[3] = ((uint8_t *)&ucHelperTime)[1];
	RingBufferStatisticsHolder[4] = ((uint8_t *)&ucHelperTime)[2];
	RingBufferStatisticsHolder[5] = ((uint8_t *)&ucHelperTime)[3];

	RingBufferRemaninzingSpace = BLE_TRANSMIT_RING_BUFFER_SIZE - (BleMainTransmitRingBuffer.Head - BleMainTransmitRingBuffer.Tail);

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

static void Sim_Create_XY_FakeMap(void)
{
	static uint32_t ExtraShifter = 0u;

	for(int i=0;  i<100; i++)
	{
		SimFakeXY_MapDat.T[i]=    HAL_GetTick();
		SimFakeXY_MapDat.X[i]=  (float)i + (float)ExtraShifter;
		SimFakeXY_MapDat.Y[i] = 0.0F + (float)ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		SimFakeXY_MapDat.T[i+100]= HAL_GetTick();
		SimFakeXY_MapDat.X[i+100]= 100.0F + (float)ExtraShifter;
		SimFakeXY_MapDat.Y[i+100] = (float)i  + (float)ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		SimFakeXY_MapDat.T[i+200]= HAL_GetTick();
		SimFakeXY_MapDat.X[i+200]= 100.0F - ((float)i*1.0F) + (float)ExtraShifter;
		SimFakeXY_MapDat.Y[i+200] =100.0F     + (float)ExtraShifter;
	}

	ExtraShifter = ExtraShifter + 5;
}

void Sim_FakeBaseDataReportTask(void)
{
	static uint32_t SimulatorFakeBaseDataTimer = 0;
	static uint16_t ProbeIterator = 0u;
	static uint8_t SyncIdIter = 1;
	static float WaveHelper = 0.0F;

	if(HAL_GetTick() - SimulatorFakeBaseDataTimer >= 20 && (true == SimulatorBaseDataStateActiv) )
	{
		SimulatorFakeBaseDataTimer = HAL_GetTick();
		for(int i =0; i<1; i++)
		{
			if(ProbeIterator == SIMULATOR_PROBES_COUNT)
			{
				Sim_Create_XY_FakeMap();
				ProbeIterator = 1U;
			}

			NewestLfDataReport.ucTimeStamp = HAL_GetTick();

			NewestLfDataReport.CurrMapData.PosX = SimFakeXY_MapDat.X[ProbeIterator];
			NewestLfDataReport.CurrMapData.PosY = SimFakeXY_MapDat.Y[ProbeIterator];

			NewestLfDataReport.CurrMapData.WhLftSp = (2 * sin( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 100 * WaveHelper));
			NewestLfDataReport.CurrMapData.WhRhtSp = (2 * sin( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 50 * WaveHelper));

			NewestLfDataReport.CurrMapData.YawRate = (2 * sin( 2 * M_PI * WaveHelper)) + (0.3 * sin( 2* M_PI * 10 * WaveHelper));


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

			NewestLfDataReport.CurrPidRegData.PidRegCorrValue = (2 * sin( 2 * M_PI * WaveHelper)) + (0.2 * sin( 2* M_PI * 13 * WaveHelper));;

			TransmitLfBaseDataReport();

			WaveHelper = WaveHelper + 0.01;
			ProbeIterator++;
			SyncIdIter++;
		}
	}

}




static void ReceiveDataHandler(void)
{
	uint8_t *MessageToRead_p = NULL;
	uint8_t MessageToReadSize= 0U;

	if(RB_Receive_Read(&BleMainReceiveRingBuffer, &MessageToReadSize,&MessageToRead_p) == RB_OK)
	{
		uint8_t *ReceivedMessageBuff = MessageToRead_p;

		BLE_MessageID_t ReceivedMessageId = ReceivedMessageBuff[0];

		switch(ReceivedMessageId)
		{
			case BLE_NvM_ErrWeigthSensorData_part1:
			{
				float ErrW1Val  = 0.0F, ErrW2Val  = 0.0F, ErrW3Val  = 0.0F,ErrW4Val  = 0.0F;
				memcpy(&ErrW1Val,  &ReceivedMessageBuff[2], sizeof(float));
				memcpy(&ErrW2Val,  &ReceivedMessageBuff[6], sizeof(float));
				memcpy(&ErrW3Val,  &ReceivedMessageBuff[10], sizeof(float));
				memcpy(&ErrW4Val,  &ReceivedMessageBuff[14], sizeof(float));
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth1_F32, ErrW1Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth2_F32, ErrW2Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth3_F32, ErrW3Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth4_F32, ErrW4Val);
				break;
			}

			case BLE_NvM_ErrWeigthSensorData_part2:
			{

				float ErrW5Val  = 0.0F, ErrW6Val = 0.0F, ErrW7Val = 0.0F, ErrW8Val  = 0.0F;
				memcpy(&ErrW5Val,  &ReceivedMessageBuff[2], sizeof(float));
				memcpy(&ErrW6Val,  &ReceivedMessageBuff[6], sizeof(float));
				memcpy(&ErrW7Val,  &ReceivedMessageBuff[10], sizeof(float));
				memcpy(&ErrW8Val,  &ReceivedMessageBuff[14], sizeof(float));
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth5_F32, ErrW5Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth6_F32, ErrW6Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth7_F32, ErrW7Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth8_F32, ErrW8Val);
				break;
			}

			case BLE_NvM_ErrWeigthSensorData_part3:
			{

				float ErrW9Val  = 0.0F,ErrW10Val = 0.0F, ErrW11Val = 0.0F,ErrWMVal  = 0.0F;
    			memcpy(&ErrW9Val,  &ReceivedMessageBuff[2], sizeof(float));
    			memcpy(&ErrW10Val,  &ReceivedMessageBuff[6], sizeof(float));
				memcpy(&ErrW11Val,  &ReceivedMessageBuff[10], sizeof(float));
				memcpy(&ErrWMVal,  &ReceivedMessageBuff[14], sizeof(float));
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth9_F32, ErrW9Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth10_F32, ErrW10Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigth11_F32, ErrW11Val);
				EE_WriteVariableF32(EE_NvmAddr_SenErrWeigthMax_F32, ErrWMVal);
				break;
			}

			case BLE_NvM_ErrWeigthSensorDataReq:
			{
//				BLE_DbgMsgTransmit("ErrWeigthReq");
				TransmitErrorWeigthData();
				break;
			}

			case BLE_NvM_PidRegDataReq:
			{

//				BLE_DbgMsgTransmit("PidRegDataReq");
				TransmitPidData();
				break;
			}

			case BLE_NvM_PidRegData:
			{
				float PidKp  = 0.0F,PidKi = 0.0F, PidKd = 0.0F,ProbeTime  = 0.0F;
    			memcpy(&PidKp,  &ReceivedMessageBuff[2], sizeof(float));
    			memcpy(&PidKi,  &ReceivedMessageBuff[6], sizeof(float));
				memcpy(&PidKd,  &ReceivedMessageBuff[10], sizeof(float));
				memcpy(&ProbeTime,  &ReceivedMessageBuff[14], sizeof(float));
				EE_WriteVariableF32(EE_NvmAddr_PidKp_F32, PidKp);
				EE_WriteVariableF32(EE_NvmAddr_PidKi_F32, PidKi);
				EE_WriteVariableF32(EE_NvmAddr_PidKd_F32, PidKd);
				EE_WriteVariableF32(EE_NvmAddr_ProbeTime_U32, ProbeTime);

				break;
			}

			case BLE_NvM_VehCfgReq:
			{
//				BLE_DbgMsgTransmit("VehCfgReq");
				TransmitVehCfgData();
				break;
			}

			case BLE_NvM_VehCfgData:
			{
				float BaseMotSpd  = 0.0F;
				uint32_t LedState = 0U, TryDetEndLine = 0U;
    			memcpy(&BaseMotSpd,  &ReceivedMessageBuff[2], sizeof(float));
    			memcpy(&LedState,  &ReceivedMessageBuff[6], sizeof(uint32_t));
				memcpy(&TryDetEndLine,  &ReceivedMessageBuff[10], sizeof(uint32_t));
				EE_WriteVariableF32(EE_NvmAddr_BaseMotorSpdValue_U32, BaseMotSpd);
				EE_WriteVariableF32(EE_NvmAddr_BlinkLadeState_U32, LedState);
				EE_WriteVariableF32(EE_NvmAddr_TryDetectEndLine_U32, TryDetEndLine);
				break;
			}

			case BLE_SimulatorStart:
			{
				SimulatorBaseDataStateActiv = true;
				break;
			}

			case BLE_SimulatorStop:
			{
				SimulatorBaseDataStateActiv = false;
				break;
			}

			default:
			{
				/*Nothing to do*/
				break;
			}

		}

		ReceivedMessageId = BLE_None;
	}

}

static BLE_CallStatus_t MessageWrite(BLE_MessageID_t MessageID,uint8_t SyncId,BLE_MessageBuffer_t *MessageData)
{
	uint8_t _tempDataTab[20] = {0};
	BLE_CallStatus_t retval = BLE_Ok;

	_tempDataTab[0] = MessageID;
	_tempDataTab[1] = SyncId;
	for(int i=0; i<BLE_DATA_FIELD_SIZE; i++)
	{
		_tempDataTab[2+i] = MessageData->BLE_Data[i];
	}

	if(RB_Transmit_Write(&BleMainTransmitRingBuffer, _tempDataTab, BLE_MIN_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLE_Error;
	}

	return retval;
}

static void TransmitDataHandler(void)
{
	static uint32_t BLE_DataTransmitWindow = 0;
	static bool TransmitWindowActiv = 0;

	static uint32_t BLE_StatisticTimer = 0;



	TransmitWindowActiv = true;
	(void)BLE_DataTransmitWindow;
/*Fajnie wyszlo na 200 wobec 60*/
	if(HAL_GetTick() - BLE_DataTransmitWindow > 350)
	{
		BLE_DataTransmitWindow = HAL_GetTick();
		TransmitWindowActiv = true;
	}

	if(HAL_GetTick() - BLE_DataTransmitWindow > 60)
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
				if(RB_Transmit_Read(&BleMainTransmitRingBuffer, &MessageToSize_p, &MessageToTransmit_p) == RB_OK)
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



/*
*********************************************************************************************
*Interface functions implementation
*********************************************************************************************
*/
void BLE_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &BleMainReceiveMessagesTab[0][0], 20);
	Sim_Create_XY_FakeMap();
}

void BLE_Task(void)
{
	Sim_FakeBaseDataReportTask();
	ReceiveDataHandler();
	TransmitDataHandler();
}

void BLE_ReportMapData(BLE_MapDataReport_t *MapData)
{
	NewestLfDataReport.CurrMapData = *MapData;
}

void BLE_ReportSensorData(BLE_SensorDataReport_t *SensorData)
{
	NewestLfDataReport.CurrSensorData = *SensorData;
}

void BLE_RegisterNvMdataUpdateInfoCallBack(void *UpdateInfoCb(void) )
{
	//TODO: to implement
}



void BLE_DbgMsgTransmit(char *DbgString, ...)
{
	static uint8_t SyncId = 0;
	uint32_t ucTimeStamp = HAL_GetTick();
	uint8_t remainingFramesCounter = 0;
	char DebugMessageBufferHelper[255] = {0};
	char DebugSingleFrameBuffer[18] = {0};
	static const uint8_t FristFrameDataSize = 13;
	static const uint8_t OtherFramesDataSize = 17;

	/*
	 * Frist Frame:
	 * |FRAMEID|SYNCID|REMAINING_FRAMES| ucTIME | DATA 13bytes
	 * Other frames:
	 * |FRAMEID|SYNCID|REMAINING_FRAMES | DATA 17bytes
	 *
	 * If remaining frames is equal "0" then desktop software
	 * should tract it as a last frame
	 *
	 * REMAINING_FRAMES counter shall be decrement with next frames
	 *
	 */

	va_list ap;
	va_start(ap, DbgString);
	uint8_t Size = vsprintf(&DebugMessageBufferHelper[0],DbgString,ap);
	va_end(ap);

	uint16_t RemSizeForAnotherFrames = 0U;

	if(Size > 13)
	{
		RemSizeForAnotherFrames = Size - FristFrameDataSize;
	}
	uint8_t DeviationRest = 0u;

	DeviationRest = RemSizeForAnotherFrames % OtherFramesDataSize;
	remainingFramesCounter = RemSizeForAnotherFrames / OtherFramesDataSize;
	if(DeviationRest > 0)
	{
		remainingFramesCounter = remainingFramesCounter + 1;
	}

	memcpy(&DebugSingleFrameBuffer[0],&remainingFramesCounter,1);
	memcpy(&DebugSingleFrameBuffer[1],&ucTimeStamp,4);

	memcpy(&DebugSingleFrameBuffer[5],&DebugMessageBufferHelper[0],FristFrameDataSize);

	BLE_MessageBuffer_t *DebugString = (BLE_MessageBuffer_t *)DebugSingleFrameBuffer;
	MessageWrite(BLE_DebugMessage,SyncId,DebugString);

	remainingFramesCounter--;
	uint8_t TransmittedFramesCounter = 0;

	while(RemSizeForAnotherFrames > 0)
	{
		memset( &DebugSingleFrameBuffer[0], 0, OtherFramesDataSize);
		memcpy(&DebugSingleFrameBuffer[0],&remainingFramesCounter,1);
		memcpy(&DebugSingleFrameBuffer[1],&DebugMessageBufferHelper[FristFrameDataSize +
																	(TransmittedFramesCounter *
																			OtherFramesDataSize)  ]
																			     ,OtherFramesDataSize);

		BLE_MessageBuffer_t *DebugString = (BLE_MessageBuffer_t *)DebugSingleFrameBuffer;
		MessageWrite(BLE_DebugMessage,SyncId,DebugString);

		TransmittedFramesCounter++;
		remainingFramesCounter--;

		RemSizeForAnotherFrames = RemSizeForAnotherFrames - OtherFramesDataSize;

		if(RemSizeForAnotherFrames > 0xFF )
		{
			//uint8 overflow detected
			//all data should be stored in ring buffer succesfully, transmission commissioned
			RemSizeForAnotherFrames = 0;
		}
		else{
			/*nothing to do*/
		}

	}

	SyncId++;;
}













