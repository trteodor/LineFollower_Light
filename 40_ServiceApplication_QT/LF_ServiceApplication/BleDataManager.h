#ifndef BLEDATAMANAGER_H
#define BLEDATAMANAGER_H

#include <QMainWindow>
#include "bluetoothleuart.h"
#include "GenericLfQCP.h"
#include "qthread.h"


class BleDataManager : public QObject
{
    Q_OBJECT
public:
    BleDataManager();

    bluetoothleUART bleConnection;

    QThread BLE_Thread;

    GenericLfQCP MapPlot;
    GenericLfQCP YawRatePlot;
    GenericLfQCP SpdPlot;

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
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        BLE_MapDataReport_t CurrMapData;
        BLE_SensorDataReport_t CurrSensorData;
    }BLE_LfDataReport_t; /*47bytes total size + 3*2 = 6*/


    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        uint16_t RingBufferRemainingSize;
        uint16_t RingBufferOverFlowCounter;
        uint16_t TransmisstedMessagesCounter;
        uint16_t RetransmissionCounter;
    }BLE_StatisticData_t;



    BLE_LfDataReport_t FullBaseData;


private:
    void BleDatMngr_BaseDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID);
    void BleDatMngr_CommunicationStatistics_Handler(const QByteArray &value);


signals:
    void BleDatMngrSignal_MapPlotUpdate(void);
    void BleDatMngrSignal_YawRatePlotUpdate(void);
    void BleDatMngrSignal_SpdPlotUpdate(void);

private slots:
    void BleDatMngr_InputHanlder(const QByteArray &value);


};

#endif // BLEDATAMANAGER_H
