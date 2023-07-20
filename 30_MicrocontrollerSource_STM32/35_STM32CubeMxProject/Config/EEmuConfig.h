


#ifndef __EE_MemMap__
#define __EE_MemMap__

/*The typedef describes virtual addresses and available Non volatile memory data variables*/

/*
 * F32 - float
 * U32 - uint32_t
 * */

typedef enum
{
	Dummy = 0xF, /*Address 0 can't texist*/
	EE_NvmAddr_SenErrWeigthSecStart, //0x10...
	EE_NvmAddr_SenErrWeigth1_F32, //0x11...
	EE_NvmAddr_SenErrWeigth2_F32,
	EE_NvmAddr_SenErrWeigth3_F32,
	EE_NvmAddr_SenErrWeigth4_F32,
	EE_NvmAddr_SenErrWeigth5_F32,
	EE_NvmAddr_SenErrWeigth6_F32,
	EE_NvmAddr_SenErrWeigth7_F32,
	EE_NvmAddr_SenErrWeigth8_F32,
	EE_NvmAddr_SenErrWeigth9_F32,
	EE_NvmAddr_SenErrWeigth10_F32,
	EE_NvmAddr_SenErrWeigth11_F32,
	EE_NvmAddr_SenErrWeigthMax_F32,
	EE_NvmAddr_SenErrWeigthSecEnd,

	EE_NvmAddr_PidKp_F32,
	EE_NvmAddr_PidKi_F32,
	EE_NvmAddr_PidKd_F32,
	
	EE_NvmAddr_ProbeTime_U32,

	EE_NvmAddr_ExpectedMotorSpdValue_F32,
	EE_NvmAddr_BlinkLadeState_U32,
	EE_NvmAddr_TryDetectEndLine_U32,
	
	EE_NvmAddr_MotAtoPwmFacLeft_U32,
	EE_NvmAddr_MotAtoPwmFacRight_U32,
	EE_NvmAddr_MotBtoPwmFacLeft_U32,
	EE_NvmAddr_MotBtoPwmFacRight_U32,

	EE_NvmAddr_EncodersOneImpDistance_F32,
	EE_NvmAddr_EncodersWheelBaseInfo_F32,

//	EE_NvmAddr_IrSensorState_U32,

	EE_NvmAddr_BleDevNamePart1_U32_,
	EE_NvmAddr_BleDevNamePart2_U32_,
	EE_NvmAddr_BleDevNamePart3_U32_,
	EE_NvmAddr_BleDevNamePart4_U32_,

	EE_NvmAddr_DevNameUpdatedFlag_U32,


	EE_VarsCount
}EE_Vars_t;


#endif // __EE_MemMap__
