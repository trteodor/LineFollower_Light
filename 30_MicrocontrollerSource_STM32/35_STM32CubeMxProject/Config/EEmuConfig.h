


#ifndef __EE_MemMap__
#define __EE_MemMap__

/*The typedef describes virtual addresses and available Non volatile memory data variables*/

/*
 * F32 - float
 * U32 - uint32_t
 * */

typedef enum
{
	Dummy = 0xFF, /*Address 0 can't texist*/
	EE_NvmAddr_SenErrWeigthSecStart, //0x100...
	EE_NvmAddr_SenErrWeigth1_F32, //0x101...
	EE_NvmAddr_SenErrWeigth2_F32,
	EE_NvmAddr_SenErrWeigth3_F32,
	EE_NvmAddr_SenErrWeigth4_F32,
	EE_NvmAddr_SenErrWeigth5_F32, /*For some reasons it dont work for me correctly why?
									- I know that i should repair the EEmu.. */
	EE_NvmAddr_SenErrWeigth6_F32,
	EE_NvmAddr_SenErrWeigth7_F32,
	EE_NvmAddr_SenErrWeigth8_F32,
	EE_NvmAddr_SenErrWeigth9_F32,
	EE_NvmAddr_SenErrWeigth10_F32,
	EE_NvmAddr_SenErrWeigth11_F32,
	EE_NvmAddr_SenErrWeigthMax_F32,
	EE_NvmAddr_SenErrWeigthSecEnd,

	EE_NvmAddr_LinePidKp_F32,
	EE_NvmAddr_LinePidKi_F32,
	EE_NvmAddr_LinePidKd_F32,

	EE_NvmAddr_ProbeTimeLinePid_U32,

	EE_NvmAddr_RightAgPidKp_F32,
	EE_NvmAddr_RightAgPidKd_F32,
	EE_NvmAddr_RightAgBaseSpeed_F32,
	EE_NvmAddr_RightAgMaxYawRate_F32,
    EE_NvmAddr_RightAgBrakeSpeedTh_F32,
    EE_NvmAddr_RightAgBrakingTime_F32,
	EE_NvmAddr_BlindAddresDueToWeirdError, /*Page size max write size? Generally I won't waste time on that...*/
    EE_NvmAddr_RightAgOriChange_F32,
    EE_NvmAddr_RightAgOriChangeAfterBrake_F32,
	EE_NvmAddr_PrTimRghtAgPid_U32,

	EE_NvmAddr_ExpectedMotorSpdValue_F32,
	EE_NvmAddr_BlinkLadeState_U32,
	EE_NvmAddr_BlackThemeFlag_U32,

	EE_NvmAddr_MotAtoPwmFacLeft_U32,
	EE_NvmAddr_MotAtoPwmFacRight_U32,
	EE_NvmAddr_MotBtoPwmFacLeft_U32,
	EE_NvmAddr_MotBtoPwmFacRight_U32,

	EE_NvmAddr_EncodersOneImpDistance_F32,
	EE_NvmAddr_EncodersWheelBaseInfo_F32,

	EE_NvmAddr_IrSensorState_U32,

	EE_NvmAddr_SpProfileEnableFlag_U32,

	EE_NvmAddr_SpPofileD01_F32,
	EE_NvmAddr_SpPofileD02_F32,
	EE_NvmAddr_SpPofileD03_F32,
	EE_NvmAddr_SpPofileD04_F32,
	EE_NvmAddr_SpPofileD05_F32,
	EE_NvmAddr_SpPofileD06_F32,
	EE_NvmAddr_SpPofileD07_F32,
	EE_NvmAddr_SpPofileD08_F32,
	EE_NvmAddr_SpPofileD09_F32,
	EE_NvmAddr_SpPofileD10_F32,
	EE_NvmAddr_SpPofileD11_F32,

	EE_NvmAddr_SpPofileBase_Sp01_F32,
	EE_NvmAddr_SpPofileBase_Sp02_F32,
	EE_NvmAddr_SpPofileBase_Sp03_F32,
	EE_NvmAddr_SpPofileBase_Sp04_F32,
	EE_NvmAddr_SpPofileBase_Sp05_F32,
	EE_NvmAddr_SpPofileBase_Sp06_F32,
	EE_NvmAddr_SpPofileBase_Sp07_F32,
	EE_NvmAddr_SpPofileBase_Sp08_F32,
	EE_NvmAddr_SpPofileBase_Sp09_F32,
	EE_NvmAddr_SpPofileBase_Sp10_F32,
	EE_NvmAddr_SpPofileBase_Sp11_F32,

	EE_NvmAddr_BluDevNamePart1_U32_,
	EE_NvmAddr_BluDevNamePart2_U32_,
	EE_NvmAddr_BluDevNamePart3_U32_,
	EE_NvmAddr_BluDevNamePart4_U32_,

	EE_NvmAddr_DevNameUpdatedFlag_U32,

	EE_VarsCount
}EE_Vars_t;


#endif // __EE_MemMap__
