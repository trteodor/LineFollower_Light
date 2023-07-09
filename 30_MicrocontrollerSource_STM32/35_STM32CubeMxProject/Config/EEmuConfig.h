


#ifndef __EE_MemMap__
#define __EE_MemMap__

/*The typedef describes virtual addresses and available Non volatile memory data variables*/

/*
 * F32 - float
 * U32 - uint32_t
 * */

typedef enum
{
	EE_NvmAddr_SenErrWeigthSecStart,
	EE_NvmAddr_SenErrWeigth1_F32,
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

	EE_NvmAddr_BaseMotorSpdValue_U32,

	EE_NvmAddr_IrSensorState_U32,
	EE_NvmAddr_BlinkLadeState_U32,
	EE_NvmAddr_TryDetectEndLine_U32,

	EE_VarsCount
}EE_Vars_t;


#endif // __EE_MemMap__
