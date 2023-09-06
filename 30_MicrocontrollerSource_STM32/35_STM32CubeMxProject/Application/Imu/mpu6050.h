#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h"
#include "mpu6050defs.h"
#include "stdint.h"
#include "stdbool.h"
#include "stm32h743xx.h"

#define MPU6050_ADDRESS 0xD0	// AD0 low
//#define MPU6050_ADDRESS 0xD1	// AD0 high



typedef struct
{
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;
    float roll;
    float pitch; 
    float yaw;
    float posX;
    float posY;
    bool flagUpdated;
}MpuData_t;



uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);


uint8_t MPU6050_GetDeviceID(void);
void MPU6050_SetDlpf(uint8_t Value);
void MPU6050_DeviceReset(uint8_t Reset);
uint8_t* MPU6050_Calibrate_Gyro(void);
void MPU6050_Start_IRQ(void);
void MPU6050_Stop_IRQ(void);



void MPU6050_Read_DMA(void);
void MPU6050_ReadDmaDataEndCallBack(MpuData_t *RecMpuData);

#endif /* MPU6050_H_ */