/*
 * EEPROM.c
 *
 *  Created on: Sep 21, 2020
 *      Author: Teodor
 */


//@@@@@@@@@@@@@@@@@@@@ HERE IS DELAY!!!! 2ms!!!

//For EEPROM M24C16
#define Sektor1 0xA0 // The memory have 16kbit i dont need the 16kBs its enough for me only the 2048bytes
					//Details in datasheet this memeory

#include "EEPROM.h"

void EEPROM_WriteEnable()
{
	//Write enable pin, in CubeMX by Default the pin is SET!!!! for my App
	HAL_GPIO_WritePin(EEPROM_WC_GPIO_Port, EEPROM_WC_Pin, GPIO_PIN_RESET);
}

void EEPROM_WriteDisable()
{
	//Write enable pin, in CubeMX by Default the pin is SET!!!!
	HAL_GPIO_WritePin(EEPROM_WC_GPIO_Port, EEPROM_WC_Pin, GPIO_PIN_SET);
}


int8_t EEPROM_WRITE(uint16_t MemAdr, uint8_t *regData , uint16_t len)
{
	// HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
	HAL_I2C_Mem_Write(&hi2c1, Sektor1, MemAdr, 1, regData, len, 1);
	return 0;

}

int8_t EEPROM_READ(uint16_t MemAdr, uint8_t *regData , uint16_t len)
{
	HAL_I2C_Mem_Read(&hi2c1, Sektor1, MemAdr, 1, regData, len, 1);
	return 0;
}

float EEPROM_WRITE_FLOAT(uint16_t MemAdr, float *regData)  //Float value is 32bit so need 4bytes in eeprom memory
{
	  union TO_EEPROM {
	      uint8_t in8bit [4];
	      uint32_t  wartosc;
	      float wartoscfloat;
	  } u;

	  	  u.wartoscfloat=*regData;

		  EEPROM_WRITE(MemAdr, (uint8_t*)&u, 4);

	return 0;
}

float EEPROM_READ_FLOAT(uint16_t MemAdr,float *regData)
{
	//Is Need some delay
	HAL_Delay(2);  //@@@@@@@@@@@@@@@@@@@@ HERE IS DELAY!!!!

	  union TO_EEPROM {
	      uint8_t in8bit [4];
	      float warf;
	  } u;

u.warf=0;

		  EEPROM_READ(MemAdr, (uint8_t*)&u, 4);

	  *regData=u.warf;

	return 0;
}

int EEPROM_WRITE_INT(uint16_t MemAdr, int *regData)  //Float value is 16bit so need 2bytes in eeprom memory
{
	  union TO_EEPROM {
	      uint8_t in8bit [4];
	      uint32_t  wartosc;
	      int warint;
	  } u;

	  	  u.warint=*regData;

		  EEPROM_WRITE(MemAdr, (uint8_t*)&u, 4);

	return 0;
}

int EEPROM_READ_INT(uint16_t MemAdr,int *regData)
{
	//Is Need some delay
	HAL_Delay(2);

	  union TO_EEPROM {
	      uint8_t in8bit [4];
	      int warint;
	  } u;

u.warint=0;

		  EEPROM_READ(MemAdr, (uint8_t*)&u, 4);

	  *regData=u.warint;

	return 0;
}




