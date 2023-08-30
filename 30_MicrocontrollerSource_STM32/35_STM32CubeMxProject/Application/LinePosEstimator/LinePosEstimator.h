#ifndef _LineSensor_H
#define _LineSensor_H



typedef enum
{
	Event_None,
	Event_RightRightAngle,
	Event_LeftRightAngle,
	Event_Cross,
}LinePosEstimatorEvent_t;



void LPE_Init();
void LPE_Task(void);

float LPE_GetPosError(void);


void LPE_RegisterLineEventCallBack(void LineEvCallback(LinePosEstimatorEvent_t LinePosEstEv) );
void LPE_RegisterDrivingAtStrightLineCallBack(void DrivStrightLineEvCallback(void) );

#endif //_LineSensor_H
