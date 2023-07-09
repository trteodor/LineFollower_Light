#ifndef BLEDATAMANAGER_H
#define BLEDATAMANAGER_H

#include <QMainWindow>
#include "bluetoothleuart.h"
#include "qmutex.h"
#include "qthread.h"

class BleDataManager : public QObject
{
    Q_OBJECT
public:
    BleDataManager();

    QThread BleDatMngr_Thread;

    bluetoothleUART bleConnection; /*Must be moved to BleDatMngr_Thread*/


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


    QMutex DebugTableScrollingBottonMutex;
    volatile bool DebugTableScrollingBottomIsActivState = false;


    typedef enum
    {
        BLE_None = 0,
        BLE_ConfirmationTag,
        BLE_CommunicationStatistics,
        BLE_BaseDataReport_part1, /*Divide most necessary data into 1 compressed buffor to effienctly communicate*/
        BLE_BaseDataReport_part2,
        BLE_BaseDataReport_part3,
        BLE_SuspendFakeProducer,
        BLE_StartFakeProducer,


        BLE_NvM_ErrWeigthSensorDataReq,
        BLE_NvM_ErrWeigthSensorDataSet,
        BLE_NvM_ErrWeigthSensorData_part1,
        BLE_NvM_ErrWeigthSensorData_part2,
        BLE_NvM_ErrWeigthSensorData_part3,

        BLE_NvM_PidRegDataReq,
        BLE_NvM_PidRegDataSet,
        BLE_NvM_PidRegData,

        BLE_NvM_VehCfgReq,
        BLE_NvM_VehCfgSet,
        BLE_NvM_VehCfgData,
    }BLE_MessageID_t;


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
        float TravelledDistance;
    }BLE_MapDataReport_t; /*Current size 6*4 = 24*/

    typedef struct
    {
        uint8_t SensorData[12];
        float PosError;
        uint8_t LastLeftLinePosConfidence;
        uint8_t LastRightLinePosConfidence;
    }BLE_SensorDataReport_t;  /*Current size= 12+4+1+1 = 18*/

    typedef struct
    {
        float PidRegCorrValue;
    }BLE_PidRegData_t; /*Current size= 4*/

    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        BLE_MapDataReport_t CurrMapData;
        BLE_SensorDataReport_t CurrSensorData;
        BLE_PidRegData_t CurrPidRegData;
    }BLE_LfDataReport_t; /*51bytes total size + 3*2 = 6|" Max 54bytes (left 3bytes)   -- stil */
                        /*However data are aligned to 4bytes... i didn't find time to solve it ;)
                          I know how to do it (very simple - packed structures but i didn't need it ;)
                          it coused that CurrPidRegData is on position 16 not 14*/

    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        uint16_t RingBufferRemainingSize;
        uint16_t RingBufferOverFlowCounter;
        uint16_t TransmisstedMessagesCounter;
        uint16_t RetransmissionCounter;
    }BLE_StatisticData_t ;


    BLE_LfDataReport_t FullBaseData;


signals:

    void BleDatMngrSignal_UpdateErrorWeigthData( float ErrW1,float ErrW2,float ErrW3,float ErrW4,float ErrW5,float ErrW6,float ErrW7,
                                                float ErrW8,float ErrW9,float ErrW10,float ErrW11,float ErrWM);

    void BleDatMngrSignal_RefreshErrorIndicatorView( uint8_t S0,uint8_t S1,uint8_t S2,uint8_t S3,uint8_t S4,uint8_t S5,
                                                   uint8_t S6,uint8_t S7,uint8_t S8,uint8_t S9,uint8_t S10,uint8_t S11,
                                                   uint8_t RightLinePosConfif,uint8_t LeftLinePosConfif,
                                                   float PosError);


    void BleDatMngrSignal_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor = QColor( 255,255,255) );
    void BleDatMngrSignal_DebugTable_ScrollToBottom();


    void BleDatMngrSignal_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
                                                            uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter);


    void BleDatMngrSignal_PlotMapUpdate(void);
    void BleDatMngrSignal_PlotYawRateUpdate(void);
    void BleDatMngrSignal_PlotSpdUpdate(void);
    void BleDatMngrSignal_PlotPosErrUpdate(void);
    void BleDatMngrSignal_PlotPidRegValUpdate(void);

    void BleDatMngrSignal_PlotMapAppendData(float PosX, float PosY);
    void BleDatMngrSignal_PlotYawRateAppendData(uint32_t FrameId, float YrValue);
    void BleDatMngrSignal_PlotSpdAppendData(uint32_t FrameId, float SpdValue);
    void BleDatMngrSignal_PlotPosErrAppendData(uint32_t FrameId, float PossErrValue);
    void BleDatMngrSignal_PlotPidRegValAppendData(uint32_t FrameId, float PidRegVal);

private slots:
    void BleDatMngr_InputHanlder(const QByteArray &value);


private:
    void BleDatMngr_BaseDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID);
    void BleDatMngr_ErrorWeigthDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID);
    void BleDatMngr_CommunicationStatistics_Handler(const QByteArray &value);
    void BleDatMngr_BaseDataInsertToDebugTable(uint32_t FrameCounter);

};

#endif // BLEDATAMANAGER_H
