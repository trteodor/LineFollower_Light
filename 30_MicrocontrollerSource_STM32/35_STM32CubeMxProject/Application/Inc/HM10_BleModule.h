#ifndef _HM10_BleModule_H
#define _HM10_BleModule_H

#include "stdint.h"

#define ReceiveBufferSize 128
#define TransmitBufferSize 128

extern uint8_t ReceiveBuffer[ReceiveBufferSize]; // Receive from UART Buffer
extern uint8_t TransmitBuffer[TransmitBufferSize]; // Message to transfer by UART
extern uint8_t Length; // Message length

typedef enum
{
	BLE_OK,
	GoToIdle,
	Idle,
	SendingDataToMobAppOneTime,
	ContinuousCyclicSendingData,
	SendDrivingTimeAndAvSpeed,
}Ble_AppStatus;

typedef enum
{
	BLE_TX_Ready,
	BLE_TX_Busy,
}BLE_TransmitState_t;

typedef struct
{
	BLE_TransmitState_t BleTxState;
	Ble_AppStatus Ble_AppSt;
	uint8_t ReceiveBuffer[ReceiveBufferSize];
	Ble_AppStatus (* ActualStateCallBack)();
}HM10BLE_t;

extern HM10BLE_t HM10BLE_App;

void HM10BLE_Init();
void HM10Ble_Task();
void HM10BLE_Tx(uint8_t *pData, uint16_t Size);
void HM10BLE_RxEventCallback(uint16_t RecDataSize);
void HM10BLE_TxCmpltEventCallback();

#endif //_HM10_BleModule_H
