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
#include "RC5_TSOP2236.h"
#include <stdlib.h>
//uint8_t TimeDiffTableD[35];

uint32_t RC5_Time;

static void (*IrRecDataInfoCbP)(uint32_t IrRecData);


RC5_Status RC5_Get_Signal(RC5Struct *RC5_Handle);

RC5_Status RC5_INIT(RC5Struct *RC5_Handle)  //nothing to init :/
{
	RC5_Handle->Status=RC5_EMPTY;
	RC5_Handle->ReadyToReadFlag=0;
	RC5_Handle->ThisObjectRC5Time=0;
	RC5_Handle->ProcessedData=malloc(sizeof(RC5_Data));
	RC5_Handle->TimeDiffTable=  malloc(35);  //allocate tab for data
	//RC5_Handle->TimeDiffTable=(uint8_t*)&TimeDiffTableD;  //allocate tab for data
return RC5_OK;
}

RC5_Status RC5_100usTimer() //This function have to be called evry 100us
{
		RC5_Time++;
	return RC5_OK;
}

RC5_Status RC5_IR_EXTI_GPIO_ReceiveAndDecodeFunction(RC5Struct *RC5_Handle)
{
	if( ( (RC5_Handle->ThisObjectRC5Time+160 )  >  RC5_Time  &&  (RC5_Handle->ThisObjectRC5Time+90 ) <   RC5_Time   )  ) //Receive Sync Header
	{
		RC5_Handle->DataTableIndex=0;

		//return RC5_IN_PROGRESS;
	}
	if(RC5_Handle->ThisObjectRC5Time +500  < RC5_Time)
			{
		RC5_Handle->DataTableIndex=0;
			}
	RC5_Handle->TimeDiffTable[RC5_Handle->DataTableIndex]= RC5_Time-(RC5_Handle->ThisObjectRC5Time);
	RC5_Handle->DataTableIndex++;

		if(RC5_Handle->TimeDiffTable[RC5_Handle->DataTableIndex] >=8 &&
							RC5_Handle->TimeDiffTable[RC5_Handle->DataTableIndex] < 15)   //Represents 0
		{
		RC5_Handle->Rc_Data &= ~(1UL << (32-RC5_Handle->DataTableIndex));
		}
		else if(RC5_Handle->TimeDiffTable[RC5_Handle->DataTableIndex] >=16 &&
							RC5_Handle->TimeDiffTable[RC5_Handle->DataTableIndex]<25) //Represents 1
		{
		RC5_Handle->Rc_Data |= (1UL << (32-RC5_Handle->DataTableIndex));
		}

   	   if(RC5_Handle->DataTableIndex==32)
   	   {
   		RC5_Handle->DataTableIndex=0;
   		RC5_Get_Signal(RC5_Handle);
   		RC5_Handle->Status=RC5_READY;
		if(IrRecDataInfoCbP != NULL)
		{
			IrRecDataInfoCbP(RC5_Handle->Rc_Data & 0x0000FFFF);
		}
   		return RC5_READY;
   	   }

   	RC5_Handle->ThisObjectRC5Time=RC5_Time;
	return RC5_IN_PROGRESS;
}

RC5_Status RC5_Get_Signal(RC5Struct *RC5_Handle)
{
		RC5_Handle->ProcessedData->rawData= RC5_Handle->Rc_Data&0x3FFF;
		RC5_Handle->ProcessedData->toggle_bit = (RC5_Handle->Rc_Data>>11)&0x1;
		RC5_Handle->ProcessedData->address_bits = (RC5_Handle->Rc_Data>>6)&0x1F;
		RC5_Handle->ProcessedData->data_bits = (RC5_Handle->Rc_Data&0x3F)+((RC5_Handle->ProcessedData->philips_bit==0x0)?0x40:0x0);

	return RC5_OK;
}

RC5_Status RC5_Read_AllReceived_Data(RC5Struct *RC5_Handle, uint32_t *Data)
{
	if(RC5_Handle->Status==RC5_READY)
	{
		*Data=RC5_Handle->Rc_Data;
		RC5_Handle->Rc_Data=0;

		RC5_Handle->Status=RC5_EMPTY;
		return RC5_OK;
	}
	if(RC5_IN_PROGRESS)
	{
				//Blink led or something
		return RC5_IN_PROGRESS;
	}

	return RC5_Error;
}
RC5_Status RC5_ReadAddresAndData(RC5Struct *RC5_Handle, uint8_t *RC5_Data, uint8_t *RC5_Adresss)
{
	if(RC5_Handle->Status==RC5_READY)
	{
		*RC5_Data=RC5_Handle->ProcessedData->data_bits;
		*RC5_Adresss=RC5_Handle->ProcessedData->address_bits;
		RC5_Handle->Rc_Data=0;
		RC5_Handle->Status=RC5_EMPTY;
		return RC5_OK;
	}
	if(RC5_IN_PROGRESS)
	{
				//Blink led or something
		return RC5_IN_PROGRESS;
	}

	return RC5_Error;
}
RC5_Status RC5_ReadNormal(RC5Struct *RC5_Handle, uint16_t *RC5_Data)
{
	if(RC5_Handle->Status==RC5_READY)
	{
		*RC5_Data=RC5_Handle->Rc_Data& 0x0000FFFF;

		RC5_Handle->Status=RC5_EMPTY;
		return RC5_OK;
	}
	if(RC5_IN_PROGRESS)
	{
				//Blink led or something
		return RC5_IN_PROGRESS;
	}

	return RC5_Error;
}

extern void RC5_RegisterCallBackNewMessage(void IrRecNewDataCb(uint32_t IrRecData) )
{
	IrRecDataInfoCbP = IrRecNewDataCb;
}
