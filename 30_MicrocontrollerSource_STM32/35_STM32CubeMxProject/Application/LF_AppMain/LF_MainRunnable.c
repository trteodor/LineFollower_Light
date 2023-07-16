
#include "BLE_Comm.h"
#include "EEmu.h"
#include "LedMngr.h"
#include "EncodersHandler.h"
#include "LinePosEstimator.h"

#include "LF_Menager.h"
#include "LF_LinePid.h"


/** @brief LF_AppTask
*  @details Called after power on cycle from main function (file main.c)
*/
void LF_AppInit(void)
{
	/*Hardware peripherials are already initialized in "main.c"*/

	EE_Init();	 /*EEPROM Emulation module init*/
	BLE_Init();  /*Bluetooth low energy module init*/
	LED_Init();  /*Led module init*/
	ENC_Init();  /*Encoder module init*/
	LPE_Init();  /*Line Position estimator init*/

	/*Line Following App functions*/
	App_LinePidInit();  /*Line Keep Pid regulator init*/
	LF_MngrInit();
}


/** @brief LF_AppTask
*   @details Called continously from an infinity loop in main function (file main.c)
*/
void LF_AppTask(void) //only one Task without any RTOS, all works fine -- for now the solution is inaf :)
{
	LED_MngrTask();
	BLE_Task();
	LPE_Task();
	ENC_Task();

	App_LinePidTask(); /*Line Keep Pid regulator task*/

	/*Highest layer Line Follower Application Task: */
	/*TODO: Temporary turned off!!!*/
	LF_MngrTask(); /*Line Following Menager task */
	/*Controll if driving or no..*/

}
