#ifndef _BLE_ServiceModule_H
#define _BLE_ServiceModule_H

#include "stdint.h"

#define ReceiveBufferSize 128
#define TransmitBufferSize 128

void BLE_Task(void);
void BLE_Tx(uint8_t *pData, uint16_t Size);
void BLE_RxEventCallback(uint16_t RecDataSize);
void BLE_TxCmpltEventCallback();

#endif //_BLE_ServiceModule_H
