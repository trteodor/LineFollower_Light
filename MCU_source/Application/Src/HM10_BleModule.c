/*
 * Part od applation which
 */
#include "main.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "EEPROM.h"
#include "EEPROM_VarLocDef.h"
#include "HM10_BleModule.h"
#include "HM_10_BleAppCommands.h"

#include "LineSensorsModule.h"
#include "PID_Reg_Module.h"
#include "ftoa_function.h"
#include "IR_Module.h"
#include "BlinkLedMod.h"
#include "LF_AppMain.h"
#include "Encoders_Module.h"

extern Robot_Cntrl_t Robot_Cntrl;

HM10BLE_t HM10BLE_App;


void HM10BLE_Tx(uint8_t *pData, uint16_t Size);
static Ble_AppStatus SendActualLineSensorDataFun();
static Ble_AppStatus SendActualErrorWeightsAndWhenLineIsDetectedFun();
static Ble_AppStatus SendActualPID_AndCalcMotor_DataFun();
static Ble_AppStatus SendActualPidSettingsFun();
static Ble_AppStatus SendActualDataFor_Adv_ScreenToM_AppFun();
//static Ble_AppStatus CreateAndSendTrackMapToMobileApp();
static void SendDrivingTimeAndAvSpeedFun();

void HM10BLE_Init()
{
	 //InitBle Listening
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, HM10BLE_App.ReceiveBuffer, ReceiveBufferSize);
}

void HM10Ble_Task()
{
	if(HM10BLE_App.Ble_AppSt == SendDrivingTimeAndAvSpeed)
	{
		SendDrivingTimeAndAvSpeedFun();
		HM10BLE_App.Ble_AppSt = Idle;
	}

		//send the Driving Time to mobile App and Average Speed
		//here i should only set flag for Ble mod, and the ble module will send
		//this data to phone... using corresponding function


	if(HM10BLE_App.ActualStateCallBack != NULL  && HM10BLE_App.Ble_AppSt != Idle)
	{
		if(HM10BLE_App.ActualStateCallBack() != BLE_OK)
		{
			//for example change state
			HM10BLE_App.ActualStateCallBack = NULL;
			HM10BLE_App.Ble_AppSt=Idle;
			return;
		}
	}
}


void HM10Ble_ExecuteCommand(HM10BleCommand_t HM10BLE_Command)
{
	switch(HM10BLE_Command)
	{
	//General App view
	case DrivingStartCommand :
	{
		Robot_Cntrl.RobotState = LF_go_Start;
	break;
	}
	case DrivingStopCommand:
	{
		Robot_Cntrl.RobotState = LF_go_Stop;
	break;
	}
	/* @@@@@ @@@@ Commands for Save data to .txt file on Phone */
	/* @@@@@ @@@@ "Basic" Screen in Mobile App */
	case StartSendActualPID_AndCalcMotor_Data:
	{
		HM10BLE_App.Ble_AppSt=ContinuousCyclicSendingData;
		HM10BLE_App.ActualStateCallBack = SendActualPID_AndCalcMotor_DataFun;
	break;
	}
	case StopSendActualPID_AndCalcMotor_Data:
	{
		HM10BLE_App.Ble_AppSt=Idle;
	break;
	}
	case SendActualPidSettings:
	{
		HM10BLE_App.Ble_AppSt = SendingDataToMobAppOneTime;
		HM10BLE_App.ActualStateCallBack = SendActualPidSettingsFun;
	break;
	}
	case SetPid_Kp:
	{
		PID_Module.Kp = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_Addr_PID_Kp,&PID_Module.Kp);
	break;
	}
	case SetPid_Kd:
	{
		PID_Module.Kd = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_Addr_PID_Kd,&PID_Module.Kd);
	break;
	}
	case SetPid_Ki:
	{
		PID_Module.Ki = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_Addr_PID_Ki,&PID_Module.Ki);
	break;
	}
	case SetBaseMotorSpeedValue:
	{
		PID_Module.BaseMotorSpeed = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_AddrBaseMotorSpeedValue,&PID_Module.BaseMotorSpeed);
	break;
	}
	/* @@@@@ @@@@ Sensor Screen in Mobile App */
	case StartSendActualSensorData:
	{
		HM10BLE_App.Ble_AppSt=ContinuousCyclicSendingData;
		HM10BLE_App.ActualStateCallBack = SendActualLineSensorDataFun;
	break;
	}
	case StopSendActualSensorData :
	{
		HM10BLE_App.Ble_AppSt=Idle;
	break;
	}
	case StopSendActualSensorDataAndSendActualErrorWeights :
	{
		HM10BLE_App.Ble_AppSt = SendingDataToMobAppOneTime;
		HM10BLE_App.ActualStateCallBack = SendActualErrorWeightsAndWhenLineIsDetectedFun;
	break;
	}
	/* Value from mobile App -  Weights of Error for each sensor */
	case SensorErW_1 :
	{
		SensorModule.SensorErrorValue[0] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW1_Addr,&SensorModule.SensorErrorValue[0]);
	break;
	}
	case SensorErW_2 :
	{
		SensorModule.SensorErrorValue[1] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW2_Addr,&SensorModule.SensorErrorValue[1]);
	break;
	}
	case SensorErW_3 :
	{
		SensorModule.SensorErrorValue[2] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW3_Addr,&SensorModule.SensorErrorValue[2]);
	break;
	}
	case SensorErW_4 :
	{
		SensorModule.SensorErrorValue[3] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW4_Addr,&SensorModule.SensorErrorValue[3]);
	break;
	}
	case SensorErW_5 :
	{
		SensorModule.SensorErrorValue[4] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW5_Addr,&SensorModule.SensorErrorValue[4]);
	break;
	}
	case SensorErW_6 :
	{
		SensorModule.SensorErrorValue[5] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW6_Addr,&SensorModule.SensorErrorValue[5]);
	break;
	}
	case SensorErW_7 :
	{
		SensorModule.SensorErrorValue[6] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW7_Addr,&SensorModule.SensorErrorValue[6]);
	break;
	}
	case SensorErW_8 :
	{
		SensorModule.SensorErrorValue[7] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW8_Addr,&SensorModule.SensorErrorValue[7]);
	}
	case SensorErW_9 :
	{
		SensorModule.SensorErrorValue[8] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW9_Addr,&SensorModule.SensorErrorValue[8]);
	}
	case SensorErW_10 :
	{
		SensorModule.SensorErrorValue[9] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW10_Addr,&SensorModule.SensorErrorValue[9]);
	}
	case SensorErW_11 :
	{
		SensorModule.SensorErrorValue[10] = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW11_Addr,&SensorModule.SensorErrorValue[10]);
	}
	case SensorErWMax :
	{
		SensorModule.SensorErrorMaxValue = atof((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_FLOAT(EEPROM_ErrW_Max_Addr,&SensorModule.SensorErrorMaxValue);
	break;
	}
	/*@@@@@ @@@@  Mobile App Screen "Adv" */
	case PrintActualDataFor_Adv_ScreenToM_App:
	{
		HM10BLE_App.Ble_AppSt = SendingDataToMobAppOneTime;
		HM10BLE_App.ActualStateCallBack = SendActualDataFor_Adv_ScreenToM_AppFun;
	break;
	}
	case PID_KdProbeTime :
	{
		PID_Module.PID_DerivativeTime = atoi((char *)HM10BLE_App.ReceiveBuffer);
		EEPROM_WRITE_INT(EEPROM_PID_KdProbeTime_Addr,&PID_Module.PID_DerivativeTime);
	break;
	}
	case IrSensor ://?
	{
//		IrModule.Ir_State = atoi((char *)HM10BLE_App.ReceiveBuffer);
//		uint32_t tmpIr_state= IrModule.Ir_State;
//		EEPROM_WRITE_INT(EEPROM_IrSensorState_Addr,(int *)&tmpIr_state);
	break;
	}
	case LedMode :
	{
		LedBlinkState = atof((char *)HM10BLE_App.ReceiveBuffer);
		uint32_t tmpLed_state = LedBlinkState;
		EEPROM_WRITE_INT(EEPROM_LedModeState_Addr,(int *)&tmpLed_state);
	break;
	}
	case TryDetectEndLineMark:
	{
		Robot_Cntrl.TryDetEndLapMarkState = atoi((char *)HM10BLE_App.ReceiveBuffer);
		uint32_t tmpLed_state = Robot_Cntrl.TryDetEndLapMarkState;
		EEPROM_WRITE_INT(EEPROM_TryDetectEndLineMark_Addr,(int *)&tmpLed_state);
		break;
	}
	case ReservAdvScr:
	{
		//to define
		break;
	}
	default:
	{
		//error or undefinded command
	break;
	}
	}
}

static void SendDrivingTimeAndAvSpeedFun()
{
	static char BuffToBLE[20];
	static char after_con_val[20];

	for (int i=0; i<20; i++)
	{
	BuffToBLE[i]=0;
	after_con_val[i]=0;
	}

	ftoa(Robot_Cntrl.RobotRunTime,after_con_val ,2);
	strcat(after_con_val, Lap_TimeVar_d);
	strcat(BuffToBLE,after_con_val );
	HM10BLE_App.BleTxState=BLE_TX_Busy;
	HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
	Robot_Cntrl.RobotRunTime=0; //to make sure to don;t send false data

	HAL_Delay(20);
	for (int i=0; i<20; i++)
	{
	BuffToBLE[i]=0;
	after_con_val[i]=0;
	}
	ftoa(Enc_Module.AverageSpeed,after_con_val ,2);
	strcat(after_con_val, Av_SpeedVar_d);
	strcat(BuffToBLE,after_con_val );
	HM10BLE_App.BleTxState=BLE_TX_Busy;
	HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));

	HAL_Delay(20);
	for (int i=0; i<20; i++)
	{
	BuffToBLE[i]=0;
	after_con_val[i]=0;
	}
	ftoa(Enc_Module.TakenDistance,after_con_val ,2);
	strcat(after_con_val, DistanceVal_d);
	strcat(BuffToBLE,after_con_val );
	HM10BLE_App.BleTxState=BLE_TX_Busy;
	HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
}

//static Ble_AppStatus CreateAndSendTrackMapToMobileApp()
//{
//	static uint8_t SEND_DATA_IN_FILE[100];
//
//	Create_XY_PositionMap();
//
//
//		uint32_t  SavedTimeFileLoc=HAL_GetTick();
//
//
//		sprintf((char *) SEND_DATA_IN_FILE,"X,Y,Dist_LWheel,Dist_RWheel (In Probe)\n\r" //EncodersProbeTime
//				"");
//		 HM10BLE_Tx(SEND_DATA_IN_FILE, sizeof(SEND_DATA_IN_FILE));
//		 HAL_Delay(50);
//		for(int i=1; i < Robot_Cntrl.SavedCountEncProbeNumerWhenRStopped; i++) //BLOCKING SENDING DATA!!!!
//		{
//
//			while(SavedTimeFileLoc+20 >  HAL_GetTick() )
//			{
//				//wait
//			}
//			SavedTimeFileLoc=HAL_GetTick();
//
//				uint8_t sizeBlM = sprintf((char *) SEND_DATA_IN_FILE,"%f,%f,%f,%f\n\r",PositionOnTrack.X[i],PositionOnTrack.Y[i],
//							Enc_Module.LeftWheelDistanceInProbe[i],Enc_Module.RightWheelDistanceInProbe[i] );
//				HM10BLE_App.BleTxState = BLE_TX_Busy;
//				 HM10BLE_Tx(SEND_DATA_IN_FILE, sizeBlM);
//		}
//
//	  return GoToIdle;
//}

static Ble_AppStatus SendActualDataFor_Adv_ScreenToM_AppFun()
{
	static int DataToSendQueue=1;
	static uint32_t SaveTime_BLE;
	static char BuffToBLE[20];
	static char after_con_val[20];
	if(HM10BLE_App.BleTxState==BLE_TX_Busy)
	{
		return BLE_OK; //wait for ready
	}
	if ( HAL_GetTick() - SaveTime_BLE >= TimeBeetweenNextDataPart)
	{
		SaveTime_BLE = HAL_GetTick();

		 for (int i=0; i<20; i++)
		 {
			 BuffToBLE[i]=0;
			 after_con_val[i]=0;
		 }

		  if (DataToSendQueue==1){
//		  itoa(IrModule.Ir_State,after_con_val ,10);
//		  strcat(after_con_val, IrSensor_d);
//		  strcat(BuffToBLE,after_con_val );
//		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==2){
		  itoa(LedBlinkState,after_con_val ,10);
		  strcat(after_con_val, LedMode_d);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==3){
		  itoa(Robot_Cntrl.TryDetEndLapMarkState,after_con_val ,10);
		  strcat(after_con_val, TextBoxTryDetEndLineMark);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==4){
		  after_con_val[0]='0'; //Reserved
		  strcat(after_con_val, ReservAdvScr_d);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue=1;
		  return GoToIdle;
		  }
	}
	return BLE_OK;
}

static Ble_AppStatus SendActualPidSettingsFun()
{
	static int DataToSendQueue=1;
	static uint32_t SaveTime_BLE;
	static char BuffToBLE[20];
	static char after_con_val[20];
	if(HM10BLE_App.BleTxState==BLE_TX_Busy)
	{
		return BLE_OK; //wait for ready
	}
	if ( HAL_GetTick() - SaveTime_BLE >= TimeBeetweenNextDataPart)
	{
		SaveTime_BLE = HAL_GetTick();

		 for (int i=0; i<20; i++)
		 {
			 BuffToBLE[i]=0;
			 after_con_val[i]=0;
		 }


		  if (DataToSendQueue==1){
		  ftoa(PID_Module.Kp,after_con_val ,2);
		  strcat(after_con_val, PID_KpComm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==2){
		  ftoa(PID_Module.Kd,after_con_val ,2);
		  strcat(after_con_val, PID_KdComm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==3){
		  ftoa(PID_Module.BaseMotorSpeed,after_con_val ,2);
		  strcat(after_con_val, BaseMotorSpeed_d);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==4){
		  ftoa(PID_Module.PID_DerivativeTime,after_con_val ,2);
		  strcat(after_con_val, PID_KdProbeTime_d);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue=1; //reset function
		  return GoToIdle;
		  }
	}
	 return BLE_OK;
}

static Ble_AppStatus SendActualPID_AndCalcMotor_DataFun()
{
	static int DataToSendQueue=1;
	static uint32_t SaveTime_BLE;
	static char BuffToBLE[20];
	static char after_con_val[20];
	if(HM10BLE_App.BleTxState==BLE_TX_Busy)
	{
		return BLE_OK; //wait for ready
	}
	if ( HAL_GetTick() - SaveTime_BLE >= TimeBeetweenNextDataPart)
	{
		SaveTime_BLE = HAL_GetTick();

		 for (int i=0; i<20; i++)
		 {
			 BuffToBLE[i]=0;
			 after_con_val[i]=0;
		 }


		  if (DataToSendQueue==1){
		  ftoa(PID_Module.PID_value,after_con_val ,2);
		  strcat(after_con_val, PID_ActualValue);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==2){
		  ftoa(PID_Module.CalculatedLeftMotorSpeed,after_con_val ,2);
		  strcat(after_con_val, LeftMotorSpeed);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==3){
		  ftoa(PID_Module.CalculatedRightMotorSpeed,after_con_val ,2);
		  strcat(after_con_val, RightMotorSpeed);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==4){
		  ftoa(SensorModule.PositionErrorValue,after_con_val ,2);
		  strcat(BuffToBLE,ActualPositionError);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==5){
		  ftoa(PID_Module.Ki_Sum,after_con_val ,2);
		  strcat(after_con_val, PID_Ki_Sum);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue=1; //Reset Function
		  return BLE_OK;
		  }
	}
	return BLE_OK;
}

static Ble_AppStatus SendActualErrorWeightsAndWhenLineIsDetectedFun()
{
	static int DataToSendQueue=1;
	static uint32_t SaveTime_BLE;
	static char BuffToBLE[20];
	static char after_con_val[20];
	if(HM10BLE_App.BleTxState==BLE_TX_Busy)
	{
		return BLE_OK; //wait for ready
	}
	if ( HAL_GetTick() - SaveTime_BLE >= TimeBeetweenNextDataPart)
	{
		SaveTime_BLE = HAL_GetTick();

		 for (int i=0; i<20; i++)
		 {
			 BuffToBLE[i]=0;
			 after_con_val[i]=0;
		 }


		  if (DataToSendQueue==1){
		  ftoa(SensorModule.SensorErrorValue[0],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_1);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==2){
		  ftoa(SensorModule.SensorErrorValue[1],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_2);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==3){
		  ftoa(SensorModule.SensorErrorValue[2],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_3);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==4){
		  ftoa(SensorModule.SensorErrorValue[3],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_4);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==5){
		  ftoa(SensorModule.SensorErrorValue[4],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_5);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==6){
		  ftoa(SensorModule.SensorErrorValue[5],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_6);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==7){
		  ftoa(SensorModule.SensorErrorValue[6],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_7);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==8){
		  ftoa(SensorModule.SensorErrorValue[7],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_8);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==9){
		  ftoa(SensorModule.SensorErrorValue[8],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_9);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==10){
		  ftoa(SensorModule.SensorErrorValue[9],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_10);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==11){
		  ftoa(SensorModule.SensorErrorValue[10],after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_11);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
		  }
		  if (DataToSendQueue==12){
		  ftoa(SensorModule.SensorErrorMaxValue,after_con_val ,2);
		  strcat(after_con_val, S_LineErWToM_App_Max);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue=1; //Reset function
		  return GoToIdle; //Transmision End
		  }
	}
	return BLE_OK;
}

static Ble_AppStatus SendActualLineSensorDataFun()
{
	static int DataToSendQueue=1;
	static uint32_t SaveTime_BLE;
	static char BuffToBLE[20];
	static char after_con_val[20];

	if(HM10BLE_App.BleTxState==BLE_TX_Busy)
	{
		return BLE_OK; //wait for ready
	}


	if ( HAL_GetTick() - SaveTime_BLE >= TimeBeetweenNextDataPart)
	{
		SaveTime_BLE = HAL_GetTick();

		 for (int i=0; i<20; i++)
		 {
			 BuffToBLE[i]=0;
			 after_con_val[i]=0;
		 }

	  if(DataToSendQueue==1){
		  itoa(SensorModule.SensorADCValues[0],after_con_val,10);
		  strcat(BuffToBLE, S_Line_1_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==2){
		  itoa(SensorModule.SensorADCValues[1],after_con_val,10);
		  strcat(BuffToBLE, S_Line_2_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==3){
		  itoa(SensorModule.SensorADCValues[2],after_con_val,10);
		  strcat(BuffToBLE, S_Line_3_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==4){
		  itoa(SensorModule.SensorADCValues[3],after_con_val,10);
		  strcat(BuffToBLE, S_Line_4_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==5){
		  itoa(SensorModule.SensorADCValues[4],after_con_val,10);
		  strcat(BuffToBLE, S_Line_5_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==6){
		  itoa(SensorModule.SensorADCValues[5],after_con_val,10);
		  strcat(BuffToBLE, S_Line_6_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==7){
		  itoa(SensorModule.SensorADCValues[6],after_con_val,10);
		  strcat(BuffToBLE, S_Line_7_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==8){
		  itoa(SensorModule.SensorADCValues[7],after_con_val,10);
		  strcat(BuffToBLE, S_Line_8_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==9){
		  itoa(SensorModule.SensorADCValues[8],after_con_val,10);
		  strcat(BuffToBLE, S_Line_9_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==10){
		  itoa(SensorModule.SensorADCValues[9],after_con_val,10);
		  strcat(BuffToBLE, S_Line_10_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==11){
		  itoa(SensorModule.SensorADCValues[10],after_con_val,10);
		  strcat(BuffToBLE, S_Line_11_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==12){
		  itoa(SensorModule.SensorADCValues[11],after_con_val,10);
		  strcat(BuffToBLE, S_Line_12_Data_ToM_App_Comm);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue++;
		  return BLE_OK;
	  }
	  if(DataToSendQueue==13){
		  ftoa(SensorModule.PositionErrorValue,after_con_val ,2);
		  strcat(BuffToBLE, ActualPositionError_SS);
		  strcat(BuffToBLE,after_con_val );
		  HM10BLE_App.BleTxState=BLE_TX_Busy;
		  HM10BLE_Tx((uint8_t *)BuffToBLE, sizeof(BuffToBLE));
		  DataToSendQueue=1; //ResetFunction
		  return BLE_OK;
	  }

	}
	return BLE_OK;
}

static uint8_t GetCommandFromBleMessage(uint16_t RecDataSize)
{
	if(HM10BLE_App.ReceiveBuffer[RecDataSize-2] == 226) //special mark for save data in file only
														//anyway you can always check MobileApp Source and UCF-8
														//Rly dk how to explain ;)
	{
		RecDataSize--;
	}
	return HM10BLE_App.ReceiveBuffer[RecDataSize-2];
}

static void PrepareBleAppModuleToNewCommand()
{
	HM10BLE_App.Ble_AppSt = Idle;
	HM10BLE_App.ActualStateCallBack = NULL;
}

void HM10BLE_RxEventCallback(uint16_t RecDataSize)
{
	PrepareBleAppModuleToNewCommand();
	HM10BleCommand_t HM10BLE_Command = GetCommandFromBleMessage(RecDataSize);
	HM10Ble_ExecuteCommand(HM10BLE_Command);

}

void HM10BLE_TxCmpltEventCallback()
{
	HM10BLE_App.BleTxState=BLE_TX_Ready;
}

void HM10BLE_Tx(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit_DMA(&huart2, pData, Size);
}


