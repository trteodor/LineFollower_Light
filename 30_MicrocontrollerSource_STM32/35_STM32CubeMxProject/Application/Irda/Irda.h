

#ifndef __IRDA_H__
#define __IRDA_H__

typedef enum
{
    IR_RobotStart,
    IR_RobotStop,
}IrCommands_t;

void HW_IR_TIM_100usElapsed(void); /* Called every 100ms by HW timer - TIM2 from IRQ */
void IR_RegisterEventCommandCb(void IrRecEventCb(IrCommands_t IrCommand));
void IR_Init(void);
void IR_Task(void);

#endif //__IRDA_H__