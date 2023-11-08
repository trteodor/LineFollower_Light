

#include "RC5_TSOP2236.h"
#include "Irda.h"
#include "BluetoothClassicComm.h"

#define IR_PIONEER_PILOT_ROBOT_START_COMMAND 0xA15E
#define IR_PIONEER_PILOT_ROBOT_STOP_COMMAND 0x32CC

static RC5Struct TSOP2236;

static void (*IrRecCommandEventCb)(IrCommands_t IrRecData);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==IR_PIN_INPUT_Pin)
	{
		RC5_IR_EXTI_GPIO_ReceiveAndDecodeFunction(&TSOP2236);
	}
}


void HW_IR_TIM_100usElapsed(void)
{
	RC5_100usTimer();
}

void IrDataReceivedCb(uint32_t RecData)
{
    if(IrRecCommandEventCb != NULL)
    {
        switch(RecData)
        {
            case IR_PIONEER_PILOT_ROBOT_START_COMMAND:
            {
                IrRecCommandEventCb(IR_RobotStart);
                break;
            }
            case IR_PIONEER_PILOT_ROBOT_STOP_COMMAND:
            {
                IrRecCommandEventCb(IR_RobotStop);
                break;
            }
        }
    }
    else
    {
        BLU_DbgMsgTransmit("Ir RecDataValue:0x%x ",RecData);
    }
}

void IR_RegisterEventCommandCb(void IrRecEventCb(IrCommands_t IrCommand))
{
    IrRecCommandEventCb = IrRecEventCb;
}

void IR_Init(void)
{
    RC5_INIT(&TSOP2236);
    RC5_RegisterCallBackNewMessage(IrDataReceivedCb);
}

void IR_Task(void)
{
    /* nothing to do */
}
