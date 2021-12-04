#ifndef INC_AppMainJobs_H_
#define INC_AppMainJobs_H_

#include "stdbool.h"
#include "Encoders_Module.h"

extern void HM10BLE_Init();
extern void LF_App_MainConfig(void);
extern void LF_App_MainTask(void);


#define CountStatesWhenLineBy4SenDetToEndLapMark 25

typedef struct
{
	float X[MaxProbeNumber];
	float Y[MaxProbeNumber];
	float T[MaxProbeNumber];
}PositionOnTrack_t;

typedef enum
{
	LF_Ok,
	LF_Idle,
	LF_go_Stop,
	LF_go_Start,
	LF_Started,
}RobotState_t;

typedef enum
{
	ResetState,
	Start_MarkDet,
	End_MarkDet,

}EndLapMarkStates_t;

typedef enum
{
	MapSt_Idle,
	MapSt_GoToCreate,
	MapSt_Created,
}TrackMapActions_t;

typedef enum
{
	NotActive,
	Active,
}TryDetectEndLapMarkState_t;


typedef struct
{
	EndLapMarkStates_t EndLapMarkStates;
	TryDetectEndLapMarkState_t TryDetEndLapMarkState;
	RobotState_t RobotState;
	TrackMapActions_t TrackMapActions;
	bool EndLapMarkDetection;
	bool IsMapAvailable;
	bool IsFlagStartedForDrivingTime;

	float RobotStartTime;
	float RobotStopTime;
	float RobotRunTime;

	uint32_t SavedCountEncProbeNumerWhenRStopped;


}Robot_Cntrl_t;

void Create_XY_PositionMap();

extern Robot_Cntrl_t Robot_Cntrl;
extern PositionOnTrack_t PositionOnTrack;


#endif /* _AppMainJobs_H */
