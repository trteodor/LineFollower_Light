/*
 * EEPROM.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Teodor
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "i2c.h"
#include "main.h"

int8_t EEPROM_WRITE(uint16_t MemAdr, uint8_t *regData , uint16_t len);

int8_t EEPROM_READ(uint16_t MemAdr, uint8_t *regData , uint16_t len);

float EEPROM_WRITE_FLOAT(uint16_t MemAdr, float *regData);

float EEPROM_READ_FLOAT(uint16_t MemAdr,float *regData);

int EEPROM_READ_INT(uint16_t MemAdr,int *regData);

int EEPROM_WRITE_INT(uint16_t MemAdr, int *regData);

void EEPROM_WriteEnable();
void EEPROM_WriteDisable();

#endif /* INC_EEPROM_H_ */
