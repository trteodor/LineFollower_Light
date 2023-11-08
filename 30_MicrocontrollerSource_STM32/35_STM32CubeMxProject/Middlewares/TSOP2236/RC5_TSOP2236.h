/*
 * TSOP2236_new_T.h
 *
 *  Created on: 20 apr. 2021
 *  trteodor@gmail.com
 *  	The Mit License
 *      Author: Teodor Rosolowski
 *      https://github.com/trteodor
 *This file can have some error because i haven't a good remote infrated pilot for test
 */
/*How use it?
*  Its easy, follow this steps:
*  1. Call "RC5_IR_EXTI_GPIO_ReceiveAndDecodeFunction" function on evry falling edge of the Input PIN. For example from External interrupt
*  2. Call The "RC5_100usTimer" function exactly every 100us
*  3. Create New global RC5Struct object or allocate memory for this object in your main function/file
*  4. Call Init Function in main with pointer to object created in step 3
*  5. Read the Received data with function "RC5_ReadNormal" -- RC5_ReadNormal(&RC5Device,&RC5_RecDat)
*/

#ifndef INC_RC5_TSOP2236_H_
#define INC_RC5_TSOP2236_H_
#include "main.h"


typedef enum{
	RC5_OK,
	RC5_Error,
	RC5_START, /**< Bits receiving starts. */
	RC5_IN_PROGRESS, /**< Receiving in progress. */
	RC5_READY, /**<  Frame is received and ready to interpret. */
	RC5_EMPTY, /**<  Any Data */
}RC5_Status;

typedef struct __RC5_Data{
	uint8_t toggle_bit; /**< A toogle bit, which toggles with each button press. */
	uint8_t address_bits; /**< A five-bit system address. */
	uint8_t data_bits; /**< A six-bit command. */
	uint8_t philips_bit; /*!< A field bit, which denotes whether the command sent is in the lower field or the upper field. A part of extended protocal version. */
	uint32_t rawData; /**< Raw value */
}RC5_Data;

typedef struct __RC5Struct{
	uint32_t Rc_Data; /**< Raw value */
	RC5_Data *ProcessedData;
	RC5_Status Status;
	uint8_t ReadyToReadFlag;
	uint8_t *TimeDiffTable;
	uint8_t DataTableIndex;
	uint32_t ThisObjectRC5Time;
}RC5Struct;


extern RC5_Status RC5_IR_EXTI_GPIO_ReceiveAndDecodeFunction(RC5Struct *RC5_Handle); //You need call this function on evry falling edge on IR PIN
extern RC5_Status RC5_100usTimer();      //This function have to be called evry 100us --lib tick
extern RC5_Status RC5_ReadNormal(RC5Struct *RC5_Handle, uint16_t *RC5_Data);
extern RC5_Status RC5_ReadAddresAndData(RC5Struct *RC5_Handle, uint8_t *RC5_Data, uint8_t *RC5_Adresss);
extern RC5_Status RC5_Read_AllReceived_Data(RC5Struct *RC5_Handle, uint32_t *Data);
extern RC5_Status RC5_INIT(RC5Struct *RC5_Handle);
extern RC5_Status RC5_100usTimer();


#endif /* INC_RC5_TSOP2236_H_ */
