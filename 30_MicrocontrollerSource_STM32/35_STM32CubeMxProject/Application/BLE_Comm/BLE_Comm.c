/*
 * Part od applation which
 */
#include "BLE_Comm.h"


#include "string.h"
#include <stdio.h>
#include <stdlib.h>

#include "dma.h" /*DMA driver part of STM32 HAL Library*/
#include "usart.h"  /*usart driver part of STM32 HAL Library*/
#include "gpio.h"  /*gpio driver part of STM32 HAL Library*/


#include "NvM_EEPROM.h"
#include "Nvm_EEPROM_MemMap.h"
#include "LinePosEstimator.h"
#include "PID_Reg_Module.h"
#include "ftoa_function.h"
#include "BlinkLedMod.h"
#include "LF_AppMain.h"
#include "Encoders_Module.h"

#define MaxProbeNumberr 300
#define DataBufferSize 20

typedef struct
{
	uint32_t X[MaxProbeNumberr];
	uint16_t Y[MaxProbeNumberr];
	uint16_t T[MaxProbeNumberr];
}PositionOnTrackL_t;

float LeftWheelSpeedSimulator_f = 5.0F;
float RightWheelSpeedSimulator_f = 3.0F;
uint8_t SyncID = 0;


PositionOnTrackL_t  PositionOnTrackL;


void Create_XY_FakeMap(void)
{
	for(int i=0;  i<100; i++)
	{
		PositionOnTrackL.T[i]=    HAL_GetTick();
		PositionOnTrackL.X[i]= 0;
		PositionOnTrackL.Y[i] = i;
	}
	for(int i=100;  i<200; i++)
	{
		PositionOnTrackL.T[i]= HAL_GetTick();
		PositionOnTrackL.X[i]= i-100;
		PositionOnTrackL.Y[i] = 100;
	}
	for(int i=200;  i<300; i++)
	{
		PositionOnTrackL.T[i]= HAL_GetTick();
		PositionOnTrackL.X[i]= 100-(i-200);
		PositionOnTrackL.Y[i] = 100;
	}
}


void BLE_Init(void)
{
	Create_XY_FakeMap();
	 /*InitBle Listening*/
	//   HAL_UARTEx_ReceiveToIdle_DMA(&huart2, BLE_App.ReceiveBuffer, ReceiveBufferSize);
}




uint8_t *CreateEncoderModuleMessage(void)
{
	static uint8_t DataBuffer[20];
	static uint16_t ProbeIterator = 0u;
	for(int i=0; i<20; i++)
	{
		DataBuffer[i] = 0U; /*sanitize data buffer*/
	}

	if(ProbeIterator == MaxProbeNumberr)
	{
		HAL_Delay(15000);
		ProbeIterator = 0U;
	}

	DataBuffer[0] = 'M';
	DataBuffer[1] = SyncID++; /*SyncID*/

	uint32_t TimeHelper = HAL_GetTick();
	uint16_t PosXHelper = PositionOnTrackL.X[ProbeIterator];
	uint16_t PosYHelper = PositionOnTrackL.Y[ProbeIterator];
	uint32_t RightWheelSpeedSimulator_u32 = (uint32_t)RightWheelSpeedSimulator_f;
	uint32_t LeftWheelSpeedSimulator_u32 = (uint32_t)LeftWheelSpeedSimulator_f;

	DataBuffer[2] = ((uint8_t *)&TimeHelper)[3];
	DataBuffer[3] = ((uint8_t *)&TimeHelper)[2];
	DataBuffer[4] = ((uint8_t *)&TimeHelper)[1];
	DataBuffer[5] = ((uint8_t *)&TimeHelper)[0];
	DataBuffer[6] = ((uint8_t *)&PosXHelper)[1];
	DataBuffer[7] = ((uint8_t *)&PosXHelper)[0];
	DataBuffer[8] = ((uint8_t *)&PosYHelper)[1];
	DataBuffer[9] = ((uint8_t *)&PosYHelper)[0];

	DataBuffer[10] =  ((uint8_t *)&RightWheelSpeedSimulator_u32)[3];
	DataBuffer[11] = ((uint8_t *)&RightWheelSpeedSimulator_u32)[2];
	DataBuffer[12] = ((uint8_t *)&RightWheelSpeedSimulator_u32)[1];
	DataBuffer[13] = ((uint8_t *)&RightWheelSpeedSimulator_u32)[0];

	DataBuffer[14] = ((uint8_t *)&LeftWheelSpeedSimulator_u32)[3];
	DataBuffer[15] = ((uint8_t *)&LeftWheelSpeedSimulator_u32)[2];
	DataBuffer[16] = ((uint8_t *)&LeftWheelSpeedSimulator_u32)[1];
	DataBuffer[17] = ((uint8_t *)&LeftWheelSpeedSimulator_u32)[0];

	DataBuffer[18] = '\n';
	DataBuffer[19] = '\r';

	ProbeIterator++;
return DataBuffer;
}

void BLE_Task(void)
{
	static uint32_t BleTimer = 0U;

	if(HAL_GetTick() - BleTimer >= 12)
	{
		BleTimer = HAL_GetTick();
		static uint8_t *DataToSend = 0U;
		DataToSend = CreateEncoderModuleMessage();

		/* (20*8) /115200 = 1.3ms/  period must be at least minimum 1.5ms*/
		HAL_UART_Transmit_DMA(&huart2, DataToSend, DataBufferSize);
	}
}

