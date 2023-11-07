#ifndef BLUETOOTHDATAMANAGER_H
#define BLUETOOTHDATAMANAGER_H

#include <QMainWindow>
#include "bluetoothclassic.h"
#include "qmutex.h"
#include "qthread.h"




class BluDataManager : public QObject
{
    Q_OBJECT
public:
    BluDataManager();
    ~BluDataManager();

    QThread BluDatMngr_Thread;

    bluetoothClassic bleutoothClassicConnection; /*Must be moved to BluDatMngr_Thread*/

    volatile bool DebugTable_BaseDataLoggingState = false;


    /*Extra variables for synchronization beetween plotting and incomming data
    * To avoid any delays
    */
    QMutex PlottingInfoMutex;
    volatile bool MapPlotPlottingState = false;
    volatile bool YawRatePlotPlottingState = false;
    volatile bool SpdPlotPlottingState = false;
    volatile bool PosErrPlotPlottingState = false;
    volatile bool PidRegValPlotPlottingState = false;
    volatile bool OrientationPlotPlottingState = false;
    volatile bool TrvDistancePlotPlottingState = false;
    volatile bool LinePosConfPlotPlottingState = false;

    QMutex DebugTableScrollingBottonMutex;
    volatile bool DebugTableScrollingBottomIsActivState = false;


    /*
 * Type definition of Common Header or message ID for Embedded software and desktop application
 * */
    typedef enum
    {
        BLU_None = 0,
        BLU_ConfirmationTag,
        BLU_DebugMessage,

        BLU_RobotStart,
        BLU_RobotStop,

        BLU_SimulatorStart,
        BLU_TrueBaseLoggingStart,
        BLU_SimuAndTrueDataLoggingStop,

        BLU_CommunicationStats,
        BLU_BaseDataReport,

        BLU_NvM_ErrWeigthSensorDataReq,
        BLU_NvM_ErrWeigthSensorData,

        BLU_NvM_LinePidRegDataReq,
        BLU_NvM_LinePidRegData,

        BLU_NvM_RightAgHndlrDataReq,
        BLU_NvM_RightAgHndlrData,

        BLU_NvM_VehCfgReq,
        BLU_NvM_VehCfgData,

        BLU_NvM_MotorsFactorsReq,
        BLU_NvM_MotorsFactorsData,

        BLU_NvM_EncoderModCfgReq,
        BLU_NvM_EncoderModCfgData,

        BLU_NvM_SpdProfileReq,
        BLU_NvM_SpdProfileData,

        BLU_NvM_ManualCntrlCommand,/* Virutal analog controller frame */

        BLU_SetNewRobotName,

    }BLU_MessageID_t;
    /*
 * Modules should report the newest data then BLE module will transmit it
 * */

    typedef struct
    {
        float WhLftSp;
        float WhRhtSp;
        float YawRate;
        float PosX;
        float PosY;
        float PosO;
        float TravelledDistance;
    }BLU_MapDataReport_t; /*Current size 6*4 = 28*/


    typedef struct
    {
        uint8_t SensorData[12];
        float PosError;
        uint8_t LastLeftLinePosConfidence;
        uint8_t LastRightLinePosConfidence;
    }BLU_SensorDataReport_t;  /*Current size= 12+4+1+1 = 18*/

    typedef struct
    {
        float PidRegCorrValue;
    }BLU_PidRegData_t; /*Current size= 4*/

    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp; //5
        BLU_MapDataReport_t CurrMapData;
        BLU_SensorDataReport_t CurrSensorData;
        BLU_PidRegData_t LinePidRegData;
    }BLU_LfDataReport_t; /*55bytes total size
                        (3*(Frame+SyncID): 54DataBytes (Left 0 free)*/

    typedef struct
    {
        uint32_t EnabledFlag;
        float TrvDistance[11];
        float BaseSpeedValue[11];
    }BLU_NvM_SpdProfileData_t ;


    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        uint16_t RingBufferRemainingSize;
        uint16_t RingBufferOverFlowCounter;
        uint16_t TransmisstedMessagesCounter;
        uint16_t RetransmissionCounter;
    }BLU_StatisticData_t ;

    BLU_LfDataReport_t FullBaseData;


signals:

    void BluDatMngrSignal_UpdateErrorWeigthData( float ErrW1,float ErrW2,float ErrW3,float ErrW4,float ErrW5,float ErrW6,float ErrW7,
                                                float ErrW8,float ErrW9,float ErrW10,float ErrW11,float ErrWM);

    void BluDatMngrSignal_UpdatePidData(float Kp,float Ki,float Kd,uint32_t ProbeTime);

    void BluDatMngrSignal_UpdateRgAngleHndlrData( float rAgPidKp, float rAgPidKd,
                                                 float rAgBaseSpd, float rAgMaxYawRate,
                                                 float rAgBrakeSpeedTh, float rAgBrakingTime,
                                                 float rAgOriChange, float rAgOriChangeAfterBrake,
                                                 uint32_t rAgProbeTime);

    void BluDatMngrSignal_UpdateVehCfgData(float ExpectedAvSpd,uint32_t BlinkLedSt, uint32_t TryDetEndLin,uint32_t IrSensorIsEnabled);

    void BluDatMngrSignal_UpdateEncoderCfgData(float OneImpDist, float WheelBase);
    void BluDatMngrSignal_UpdateSpeedProfileData(BLU_NvM_SpdProfileData_t SpdProfileData);
    void BluDatMngrSignal_UpdateMotorsFactors(uint32_t FacA_Lft, uint32_t FacA_Rgt,uint32_t FacB_Lft,uint32_t FacB_Rht);

    void BluDatMngrSignal_RefreshErrorIndicatorView( uint8_t S0,uint8_t S1,uint8_t S2,uint8_t S3,uint8_t S4,uint8_t S5,
                                                   uint8_t S6,uint8_t S7,uint8_t S8,uint8_t S9,uint8_t S10,uint8_t S11,
                                                   uint8_t RightLinePosConfif,uint8_t LeftLinePosConfif,
                                                   float PosError);


    void BluDatMngrSignal_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor = QColor( 255,255,255) );
    void BluDatMngrSignal_DebugTable_ScrollToBottom();


    void BluDatMngrSignal_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
                                                            uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter);


    void BluDatMngrSignal_UpdateOrientation(float Orientation);

    void BluDatMngrSignal_PlotMapUpdate(void);
    void BluDatMngrSignal_PlotYawRateUpdate(void);
    void BluDatMngrSignal_PlotSpdUpdate(void);
    void BluDatMngrSignal_PlotPosErrUpdate(void);
    void BluDatMngrSignal_PlotPidRegValUpdate(void);
    void BluDatMngrSignal_PlotOrientationReplot(void);
    void BluDatMngrSignal_PlotTrvDistanceReplot(void);
    void BluDatMngrSignal_PlotPosConfidenceReplot(void);


    void BluDatMngrSignal_PlotMapAppendData(float PosX, float PosY);
    void BluDatMngrSignal_PlotYawRateAppendData(uint32_t FrameId, float YrValue);
    void BluDatMngrSignal_PlotSpdAppendData(uint32_t FrameId, float SpdValueLeftWh,float SpdValueRightWh);
    void BluDatMngrSignal_PlotPosErrAppendData(uint32_t FrameId, float PossErrValue);
    void BluDatMngrSignal_PlotPidRegValAppendData(uint32_t FrameId, float PidRegVal);
    void BluDatMngrSignal_PlotOrientationAppendData(uint32_t FrameId, float Orientation);
    void BluDatMngrSignal_PlotTrvDistanceAppendData(uint32_t FrameId, float TrvDistance);
    void BluDatMngrSignal_PlotPosConfidenceAppendData(uint32_t FrameId, uint8_t LeftPosConf, uint8_t RightPosConf);



private slots:
    void BluDatMngr_InputHanlder( char* data, uint32_t Size);


private:

    void BluDatMngr_DebugMessagerHandler(char *data,uint32_t size, BLU_MessageID_t BLE_MessID);
    void BluDatMngr_BaseDataHandler(char *data,uint32_t Size);
    void BluDatMngr_ErrorWeigthDataHandler(char *data,uint32_t Size);
    void BluDatMngr_CommunicationStatistics_Handler(char *data,uint32_t Size);
    void BluDatMngr_BaseDataInsertToDebugTable(uint32_t FrameCounter);
    void BluDatMngr_PidDataHandler(char* data, uint32_t Size);
    void BluDatMngr_RightAngleDataHandler( char* data, uint32_t Size);
    void BluDatMngr_VehCfgDataHandler(char* data, uint32_t Size);
    void BluDatMngr_MotorsFactorsDataHandler(char* data, uint32_t Size);
    void BluDatMngr_EncodersCfgDataHandler(char* data, uint32_t Size);
    void BluDatMngr_SpeedProfileDataHandler(char* data, uint32_t Size);
    void BluDatMngr_SpeedProfileHandler( char* data, uint32_t Size);

};

#endif // BLUETOOTHDATAMANAGER_H
