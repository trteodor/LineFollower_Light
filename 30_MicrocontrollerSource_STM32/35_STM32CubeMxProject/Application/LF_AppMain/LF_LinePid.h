#ifndef INC_LF_LinePid_H_
#define INC_LF_LinePid_H_


void App_LinePidInit(void);
void App_LinePidTask(void);


void App_LinePidGetComputedPwmVals(int *LeftWhExpectedPwm, int *RightWhExpectedPwm);
void App_LinePidComputeExpectedPwmValuesForExpSpd(float ExpectedSpeed,int *LeftWhExpectedPwm, int *RightWhExpectedPwm);

#endif //_LF_LinePid_H_
