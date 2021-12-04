#ifndef INC_IR_Module_H
#define INC_IR_Module_H

#define DVD_RobotStart_But 0xA15E
#define TV_RobotStop_But 0x30CE


typedef enum
{
	Ir_Idle,
	Ir_Working,
}Ir_State_t;

typedef struct
{
	uint32_t ExtraField; //for align data
	Ir_State_t Ir_State;

}IrModule_t;

extern IrModule_t IrModule;

void IR_Task();
void IR_ModInit();

#endif //INC_IR_Module_H
