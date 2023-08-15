
#include "EEmu.h"
#include "LedMngr.h"
#include "EncodersHandler.h"
#include "LinePosEstimator.h"

#include "LF_Menager.h"
#include "BluetoothClassicComm.h"


/** @brief LF_AppTask
*  @details Called after power on cycle from main function (file main.c)
*/
void LF_AppInit(void)
{
	/*Hardware peripherials are already initialized in "main.c"*/

	EE_Init();	 /*EEPROM Emulation module init*/
	BLU_Init();  /*Bluetooth low energy module init*/
	LED_Init();  /*Led module init*/
	ENC_Init();  /*Encoder module init*/
	LPE_Init();  /*Line Position estimator init*/

	/*Line Following App functions*/
	LF_MngrInit();
}


/** @brief LF_AppTask
*   @details Called continously from an infinity loop in main function (file main.c)
*/
void LF_AppTask(void) //only one Task without any RTOS, all works fine -- for now the solution is inaf :)
{
	LED_MngrTask();
	BLU_Task();
	LPE_Task();
	ENC_Task();

	/*Highest layer Line Follower Application Task: */
	LF_MngrTask(); /*Line Following Menager task */
}
