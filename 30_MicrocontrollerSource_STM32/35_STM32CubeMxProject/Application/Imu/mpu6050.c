#include "mpu6050.h"
#include "math.h"


/**
 * *******************************************************************************************
 * local defines
 * *******************************************************************************************
 * */

#define I2C_TIMEOUT 1000
#define MPU_ERR_SAMPLING_COUNTER	10000// max: 65536

#define CONF_SAMPLE_FREQ 0.005F

#define acc     0
#define gyro    1
#define X       0
#define Y       1
#define Z       2

/**
 * *******************************************************************************************
 * local type defs
 * *******************************************************************************************
 * */


/**
 * *******************************************************************************************
 * Static variables 
 * *******************************************************************************************
 * */

const float mpuScale[] = {16384.0f, 131.072f}; // acc, gyro
float mpuDataScaled[2][3];
uint8_t mpu_buffer[14];
I2C_HandleTypeDef *i2c;

/**
 * *******************************************************************************************
 * Static function prototypes 
 * *******************************************************************************************
 * */

static float invSqrt(float x);

static void Set_Calibrate_Gyro(uint8_t *data);

/**
 * *******************************************************************************************
 * static function implementation
 * *******************************************************************************************
 * */

/*!
 ************************************************************************************************
 * \brief GetEulerAngles_MadgwickFilter
 * \details 
 * \param in
 * \param out
 * \param out
 * \param out
 *
 * */
static void GetEulerAngles_MadgwickFilter(float *roll, float *pitch, float *yaw, const float invSampleFreq)//float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq, float* roll_IMU, float* pitch_IMU, float* yaw_IMU) {
{
	static float q0 = 1.0f; //initialize quaternion for madgwick filter
	static float q1 = 0.0f;
	static float q2 = 0.0f;
	static float q3 = 0.0f;
	static float B_madgwick = 0.04;  //Madgwick filter parameter

	//DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
	/*
	* See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
	* available (for example when using the recommended MPU6050 IMU for the default setup).
	*/
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	//Convert gyroscope degrees/sec to radians/sec
	mpuDataScaled[gyro][X] *= 0.0174533f;
	mpuDataScaled[gyro][Y] *= 0.0174533f;
	mpuDataScaled[gyro][Z] *= 0.0174533f;

	//Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * mpuDataScaled[gyro][X] - q2 * mpuDataScaled[gyro][Y] - q3 * mpuDataScaled[gyro][Z]);
	qDot2 = 0.5f * (q0 * mpuDataScaled[gyro][X] + q2 * mpuDataScaled[gyro][Z] - q3 * mpuDataScaled[gyro][Y]);
	qDot3 = 0.5f * (q0 * mpuDataScaled[gyro][Y] - q1 * mpuDataScaled[gyro][Z] + q3 * mpuDataScaled[gyro][X]);
	qDot4 = 0.5f * (q0 * mpuDataScaled[gyro][Z] + q1 * mpuDataScaled[gyro][Y] - q2 * mpuDataScaled[gyro][X]);

	//Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((mpuDataScaled[acc][X] == 0.0f) && (mpuDataScaled[acc][Y] == 0.0f) && (mpuDataScaled[acc][Z] == 0.0f))) {
		//Normalise accelerometer measurement
		recipNorm = invSqrt(mpuDataScaled[acc][X] * mpuDataScaled[acc][X] + mpuDataScaled[acc][Y] * mpuDataScaled[acc][Y] + mpuDataScaled[acc][Z] * mpuDataScaled[acc][Z]);
		mpuDataScaled[acc][X] *= recipNorm;
		mpuDataScaled[acc][Y] *= recipNorm;
		mpuDataScaled[acc][Z] *= recipNorm;

		//Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		//Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * mpuDataScaled[acc][X] + _4q0 * q1q1 - _2q1 * mpuDataScaled[acc][Y];
		s1 = _4q1 * q3q3 - _2q3 * mpuDataScaled[acc][X] + 4.0f * q0q0 * q1 - _2q0 * mpuDataScaled[acc][Y] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * mpuDataScaled[acc][Z];
		s2 = 4.0f * q0q0 * q2 + _2q0 * mpuDataScaled[acc][X] + _4q2 * q3q3 - _2q3 * mpuDataScaled[acc][Y] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * mpuDataScaled[acc][Z];
		s3 = 4.0f * q1q1 * q3 - _2q1 * mpuDataScaled[acc][X] + 4.0f * q2q2 * q3 - _2q2 * mpuDataScaled[acc][Y];
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		//Apply feedback step
		qDot1 -= B_madgwick * s0;
		qDot2 -= B_madgwick * s1;
		qDot3 -= B_madgwick * s2;
		qDot4 -= B_madgwick * s3;
	}

	//Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	//Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	//compute angles
	*roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	*pitch = -asin(-2.0f * (q1*q3 - q0*q2));
	*yaw = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
}

//Fast inverse sqrt for madgwick filter
static float invSqrt(float x) {
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

static void Set_Calibrate_Gyro(uint8_t *data)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH , 1, data, 6, I2C_TIMEOUT);
}

/*!
 ************************************************************************************************
 * \brief ScaleReceivedData_DMA
 * \details Function used to scale received data
 * \param 
 * \param out 
 * \param out 
 *
 * */
void ScaleReceivedData_DMA()
{
	for(uint8_t coordinat = 0; coordinat < 2; coordinat++)
	{
		for(uint8_t axis = 0; axis < 3; axis++)
		{
		mpuDataScaled[coordinat][axis] = ((float)((int16_t)((((int16_t)mpu_buffer[(coordinat*8)+(axis*2)]) << 8) | mpu_buffer[(coordinat*8)+(axis*2)+1])) / mpuScale[coordinat]);// - mpuErr[coordinat][axis];
		}
	}	
}

void GetGyroscopeRAW(int16_t *gyroRaw)//int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	gyroRaw[X] = (((int16_t)tmp[0]) << 8) | tmp[1];
	gyroRaw[Y] = (((int16_t)tmp[2]) << 8) | tmp[3];
	gyroRaw[Z] = (((int16_t)tmp[4]) << 8) | tmp[5];
}

/**
 * *******************************************************************************************
 * public function implementation
 * *******************************************************************************************
 * */

//
//	Initialization
//
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	i2c = hi2c;

	if(MPU6050_GetDeviceID() != MPU6050_ADDRESS) return 0;

	uint8_t tmp;

	tmp = 0x00;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	HAL_Delay(100);
	tmp = 0x03;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	
	tmp = 0x03;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp = 0x04;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 1, &tmp, 1, I2C_TIMEOUT);
	tmp = 0x00;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT); // +-250degree/s
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT); // +_2g

	tmp = 0x30;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);

	return 1;
}

uint8_t MPU6050_GetDeviceID(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp<<1;
}



void MPU6050_SetDlpf(uint8_t Value)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Value & 0x7);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

// void MPU6050_SetSleepEnabled(uint8_t Enable)
// {
// 	uint8_t tmp;
// 	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
// 	tmp &= ~(1<<MPU6050_PWR1_SLEEP_BIT);
// 	tmp |= ((Enable & 0x1) << MPU6050_PWR1_SLEEP_BIT);
// 	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
// }


// void MPU6050_SetLowPowerWakeUpFrequency(uint8_t Frequency)
// {
// 	uint8_t tmp;
// 	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
// 	tmp &= 0x3F;
// 	tmp |= (Frequency & 0x3) << MPU6050_PWR2_LP_WAKE_CTRL_BIT;
// 	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
// }

// void MPU6050_AccelerometerAxisStandby(uint8_t XA_Stby, uint8_t YA_Stby, uint8_t ZA_Stby)
// {
// 	uint8_t tmp;
// 	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
// 	tmp &= 0xC7;
// 	tmp |= ((XA_Stby&0x1) << MPU6050_PWR2_STBY_XA_BIT)|((YA_Stby&0x1) << MPU6050_PWR2_STBY_YA_BIT)|((ZA_Stby&0x1) << MPU6050_PWR2_STBY_ZA_BIT) ;
// 	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
// }

// void MPU6050_GyroscopeAxisStandby(uint8_t XG_Stby, uint8_t YG_Stby, uint8_t ZG_Stby)
// {
// 	uint8_t tmp;
// 	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
// 	tmp &= 0xF8;
// 	tmp |= ((XG_Stby&0x1) << MPU6050_PWR2_STBY_XG_BIT)|((YG_Stby&0x1) << MPU6050_PWR2_STBY_YG_BIT)|((ZG_Stby&0x1) << MPU6050_PWR2_STBY_ZG_BIT) ;
// 	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
// }

uint8_t* MPU6050_Calibrate_Gyro(void)
{
	static uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t gyroBias[3] = {0, 0, 0};
	int16_t gyroRaw[3];

	for(uint16_t c = 0; c < MPU_ERR_SAMPLING_COUNTER; c++)
	{
		GetGyroscopeRAW(gyroRaw);
		gyroBias[X] += gyroRaw[X];
		gyroBias[Y] += gyroRaw[Y];
		gyroBias[Z] += gyroRaw[Z];
		HAL_Delay(5);
	}
	
	gyroBias[X] /= MPU_ERR_SAMPLING_COUNTER;
	gyroBias[Y] /= MPU_ERR_SAMPLING_COUNTER;
	gyroBias[Z] /= MPU_ERR_SAMPLING_COUNTER;

	data[0] = (-gyroBias[X]/4  >> 8) & 0xFF;
	data[1] = (-gyroBias[X]/4)       & 0xFF;
	data[2] = (-gyroBias[Y]/4  >> 8) & 0xFF;
	data[3] = (-gyroBias[Y]/4)       & 0xFF;
	data[4] = (-gyroBias[Z]/4  >> 8) & 0xFF;
	data[5] = (-gyroBias[Z]/4)       & 0xFF;

	Set_Calibrate_Gyro(data);

	return data;
}


void MPU6050_Start_IRQ(void) //Enable Int
{
	uint8_t tmp = 0x01;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void MPU6050_Stop_IRQ(void) //Disable_Int
{
	uint8_t tmp = 0x00;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);

}






void MPU6050_Read_DMA(void)
{
	HAL_I2C_Mem_Read_DMA(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, mpu_buffer, 14);
}

void MPU6050_ReadDmaDataEndCallBack(MpuData_t *RecMpuData)
{
	ScaleReceivedData_DMA();
	// Madgwick Orientation Filter with 6 degrees of freedom
	GetEulerAngles_MadgwickFilter(&RecMpuData->roll, &RecMpuData->pitch, &RecMpuData->yaw, CONF_SAMPLE_FREQ);

	RecMpuData->accX = mpuDataScaled[gyro][X];
	RecMpuData->accY = mpuDataScaled[gyro][Y];
	RecMpuData->accZ = mpuDataScaled[gyro][Z];
	RecMpuData->accX = mpuDataScaled[acc][X];
	RecMpuData->accY = mpuDataScaled[acc][Y];
	RecMpuData->accZ = mpuDataScaled[acc][Z];

	RecMpuData->flagUpdated = true;
}