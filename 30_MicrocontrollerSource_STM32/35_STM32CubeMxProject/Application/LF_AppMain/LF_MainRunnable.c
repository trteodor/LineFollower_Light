
#include "EEmu.h"
#include "LedMngr.h"
#include "EncodersHandler.h"
#include "LinePosEstimator.h"

#include "LF_Menager.h"
#include "BluetoothClassicComm.h"

#include "stm32h7xx_it.h"

#define MEAS_ENABLE 1
#define MEAS_DISABLE 0
#define MEASURE_EXECUTION_TIME MEAS_DISABLE
/*Supposdly if i did it correctly in cases when i sending few longest exeuction time has been around ~0.5ms
* If you will send much more logs then usage RTOS or other time managment strategies may be required
*/

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
#if MEASURE_EXECUTION_TIME == MEAS_ENABLE
	static uint32_t ExecutionTimerLogIntervalTimer = 0U;
	static uint32_t ExectionMeasureTimerStart=0,ExectionMeasureTimerEnd = 0;
	static uint32_t LongestExecutionTime = 0;
	static uint32_t LongestExecutionTimeStamp = 0;
	static float AverageExecTime = 0.0F;
	static uint32_t LoopCounter = 0;
	ExectionMeasureTimerStart = HAL_GetTick100us();
	LoopCounter++;
#endif //MEASURE_EXECUTION_TIME

	{
		{
			LED_MngrTask();
			BLU_Task();
			
			LPE_Task();
			ENC_Task();
			
			/*Highest layer Line Follower Application Task: */
			LF_MngrTask(); /*Line Following Menager task */
		}
	}

#if MEASURE_EXECUTION_TIME == MEAS_ENABLE
	ExectionMeasureTimerEnd = HAL_GetTick100us();
	if( (ExectionMeasureTimerEnd - ExectionMeasureTimerStart) >= LongestExecutionTime){
		LongestExecutionTime = (ExectionMeasureTimerEnd - ExectionMeasureTimerStart);
		LongestExecutionTimeStamp = HAL_GetTick100us();
	}

	AverageExecTime = AverageExecTime + ( (1.0F/LoopCounter) * 
									(ExectionMeasureTimerEnd - ExectionMeasureTimerStart)  - ((AverageExecTime) ) );
	
	if(HAL_GetTick() - ExecutionTimerLogIntervalTimer > 5000){
		ExecutionTimerLogIntervalTimer = HAL_GetTick();
		BLU_DbgMsgTransmit("Longesst ExecTime: %.2fms TimeStamp:%.2fms Average: %f",(float)LongestExecutionTime/10,(float)LongestExecutionTimeStamp/10, AverageExecTime);
		LongestExecutionTime = 0;
		LoopCounter = 0;
		AverageExecTime = 0;
	}
#endif // MEASURE_EXECUTION_TIME
}
