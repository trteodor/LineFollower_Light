#include "bluetoothDataManager.h"
#include "qthread.h"

BleDataManager::BleDataManager()
{
    this->moveToThread(&BleDatMngr_Thread);
    bleutoothClassicConnection.moveToThread((&BleDatMngr_Thread)) ;

    connect(&bleutoothClassicConnection, SIGNAL(bluetoothSignalNewDataReceived(char *, uint32_t)), this, SLOT(BleDatMngr_InputHanlder( char*, uint32_t) ));

    BleDatMngr_Thread.start();
}

BleDataManager::~BleDataManager()
{
    BleDatMngr_Thread.terminate();
}

/*****************************************/
/*Tools functions start for Main Window and BLE*/
/**/
uint32_t ConvToUint32(char *data)
{
    return (((uint8_t)data[0] ) | ( (uint8_t)data[1] << 8) \
            | ( (uint8_t)data[2] << 16) | ( (uint8_t)data[3] << 24));

}
/**************************************************************/
uint32_t ConvToUint16(char *data)
{
    return (((uint8_t)data[0] ) | ( (uint8_t)data[1] << 8) );
}
/**************************************************************/
float ieee_uint32_AsBitsTo_float32(uint32_t f)
{
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}
/*Tools functions end */


void BleDataManager::BleDatMngr_CommunicationStatistics_Handler(char *data,uint32_t Size)
{
    (void)Size;
    static BleDataManager::BLE_StatisticData_t StatisticData = {0};
    static uint8_t PreviousSyncId = 255U;

//    qDebug() << "StatisticHandler SyncId" <<  ((uint8_t)value.at(1)) ;

    StatisticData.SyncId = ((uint8_t)data[1]) ;
    StatisticData.ucTimeStamp = ConvToUint32(&data[2]);
    StatisticData.RingBufferRemainingSize = ConvToUint16(&data[6]);
    StatisticData.RingBufferOverFlowCounter = ConvToUint16(&data[8]);
    StatisticData.TransmisstedMessagesCounter = ConvToUint16(&data[10]);
    StatisticData.RetransmissionCounter = ConvToUint16(&data[12]);


    if(PreviousSyncId != StatisticData.SyncId)
    {
        emit BleDatMngrSignal_CommunicationStatisticsUpdate(StatisticData.ucTimeStamp,StatisticData.RingBufferRemainingSize,StatisticData.RingBufferOverFlowCounter,
                                                            StatisticData.TransmisstedMessagesCounter,StatisticData.RetransmissionCounter);
    }

    PreviousSyncId = StatisticData.SyncId;
}



void BleDataManager::BleDatMngr_BaseDataInsertToDebugTable(uint32_t FrameCounter)
{

        QString RgtWhlSpdSimu_s = QString::number(BleDataManager::FullBaseData.CurrMapData.WhLftSp,'f',3);
        QString LftWhlSpdSimu_s = QString::number(BleDataManager::FullBaseData.CurrMapData.WhRhtSp,'f',3);

        QString BaseMapData = QString("MapDat: pX: %1 |pY: %2 pO: %3 |L_sp: %4 |R_Sp: %5| Yr: %6")
                                  .arg(BleDataManager::FullBaseData.CurrMapData.PosX)
                                  .arg(BleDataManager::FullBaseData.CurrMapData.PosY)
                                  .arg(BleDataManager::FullBaseData.CurrMapData.PosO)
                                  .arg(LftWhlSpdSimu_s).arg(RgtWhlSpdSimu_s)
                                  .arg(BleDataManager::FullBaseData.CurrMapData.YawRate) ;

        QString BaseSensorDataP1 = QString("SenDat1: S0: %1 |S1: %2 |S2: %3 |S3: %4 |S4: %5 |S5: %6 |Err: %7")
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[0])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[1])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[2])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[3])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[4])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[5])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.PosError);

        QString BaseSensorDataP2 = QString("SenDat2: S6: %1 |S7: %2 |S8: %3 |S9: %4 |S10: %5 |S11: %6 |Err: %7")
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[6])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[7])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[8])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[9])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[10])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[11])
                                       .arg(BleDataManager::FullBaseData.CurrSensorData.PosError);

//        QColor RowColor = QColor(255,255,255); /*Default row color (not modify color*/
        emit BleDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BleDataManager::FullBaseData.SyncId,BaseMapData);
        emit BleDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BleDataManager::FullBaseData.SyncId,BaseSensorDataP1);
        emit BleDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BleDataManager::FullBaseData.SyncId,BaseSensorDataP2);

        FrameCounter++;
}


void BleDataManager::BleDatMngr_BaseDataHandler(char *data,uint32_t Size)
{
    (void)Size;
    static uint32_t PrevSyncId = 255U;
    static uint32_t FullFrameCounter = 0;
    uint8_t _inputSyncId = ((uint8_t)data[1]);

    FullBaseData.SyncId = _inputSyncId;
    FullBaseData.ucTimeStamp = ConvToUint32(&data[2]);

    FullBaseData.CurrMapData.WhLftSp = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6])) ;
    FullBaseData.CurrMapData.WhRhtSp =ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[10])) ;
    FullBaseData.CurrMapData.YawRate =ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[14])) ;
    FullBaseData.CurrMapData.PosX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[18])) ;
    FullBaseData.CurrMapData.PosY =ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[22])) ;
    FullBaseData.CurrMapData.PosO =ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[26])) ;
    FullBaseData.CurrMapData.TravelledDistance =ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[30])) ;

    FullBaseData.CurrSensorData.SensorData[0] =((uint8_t)data[34]) ;
    FullBaseData.CurrSensorData.SensorData[1] =((uint8_t)data[35]) ;
    FullBaseData.CurrSensorData.SensorData[2] =((uint8_t)data[36]) ;
    FullBaseData.CurrSensorData.SensorData[3] =((uint8_t)data[37]) ;
    FullBaseData.CurrSensorData.SensorData[4] =((uint8_t)data[38]) ;
    FullBaseData.CurrSensorData.SensorData[5] =((uint8_t)data[39]) ;
    FullBaseData.CurrSensorData.SensorData[6]  =((uint8_t)data[40]) ;
    FullBaseData.CurrSensorData.SensorData[7]  =((uint8_t)data[41]) ;
    FullBaseData.CurrSensorData.SensorData[8]  =((uint8_t)data[42]) ;
    FullBaseData.CurrSensorData.SensorData[9]  =((uint8_t)data[43]) ;
    FullBaseData.CurrSensorData.SensorData[10] =((uint8_t)data[44]) ;
    FullBaseData.CurrSensorData.SensorData[11] =((uint8_t)data[45]) ;
    FullBaseData.CurrSensorData.PosError =ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[46]));
    FullBaseData.CurrSensorData.LastLeftLinePosConfidence =((uint8_t)(data[50])) ;
    FullBaseData.CurrSensorData.LastRightLinePosConfidence =((uint8_t)(data[51])) ;

    FullBaseData.LinePidRegData.PidRegCorrValue = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[52]) );

    /*56*/
    FullFrameCounter++;


    emit BleDatMngrSignal_UpdateOrientation(FullBaseData.CurrMapData.PosO);

    if(true == DebugTable_BaseDataLoggingState)
    {
    BleDatMngr_BaseDataInsertToDebugTable(FullFrameCounter);
    }

    emit BleDatMngrSignal_RefreshErrorIndicatorView( FullBaseData.CurrSensorData.SensorData[0],
                                                        FullBaseData.CurrSensorData.SensorData[1],
                                                        FullBaseData.CurrSensorData.SensorData[2],
                                                        FullBaseData.CurrSensorData.SensorData[3],
                                                        FullBaseData.CurrSensorData.SensorData[4],
                                                        FullBaseData.CurrSensorData.SensorData[5],
                                                        FullBaseData.CurrSensorData.SensorData[6],
                                                        FullBaseData.CurrSensorData.SensorData[7],
                                                        FullBaseData.CurrSensorData.SensorData[8],
                                                        FullBaseData.CurrSensorData.SensorData[9],
                                                        FullBaseData.CurrSensorData.SensorData[10],
                                                        FullBaseData.CurrSensorData.SensorData[11],
                                                        FullBaseData.CurrSensorData.LastRightLinePosConfidence,
                                                        FullBaseData.CurrSensorData.LastLeftLinePosConfidence,
                                                        FullBaseData.CurrSensorData.PosError);

    emit BleDatMngrSignal_PlotMapAppendData(FullBaseData.CurrMapData.PosX,FullBaseData.CurrMapData.PosY);
    emit BleDatMngrSignal_PlotYawRateAppendData(FullFrameCounter,FullBaseData.CurrMapData.YawRate);
    emit BleDatMngrSignal_PlotSpdAppendData(FullFrameCounter,FullBaseData.CurrMapData.WhLftSp,FullBaseData.CurrMapData.WhRhtSp);
    emit BleDatMngrSignal_PlotPosErrAppendData(FullFrameCounter,FullBaseData.CurrSensorData.PosError);
    emit BleDatMngrSignal_PlotPidRegValAppendData(FullFrameCounter,FullBaseData.LinePidRegData.PidRegCorrValue);


    if( (false == MapPlotPlottingState) && (false == YawRatePlotPlottingState) && (false == SpdPlotPlottingState)
        && (false == DebugTableScrollingBottomIsActivState) && (false == PosErrPlotPlottingState) && (false == PidRegValPlotPlottingState) )
    {
        PlottingInfoMutex.lock();
        MapPlotPlottingState = true;
        YawRatePlotPlottingState = true;
        SpdPlotPlottingState = true;
        PosErrPlotPlottingState = true;
        PidRegValPlotPlottingState = true;
        PlottingInfoMutex.unlock();


        if(true == DebugTable_BaseDataLoggingState)
        {
            DebugTableScrollingBottonMutex.lock();
            emit BleDatMngrSignal_DebugTable_ScrollToBottom();
            DebugTableScrollingBottonMutex.unlock();
        }

        emit BleDatMngrSignal_PlotMapUpdate(); /*Move plotting to MainWindow process*/
        emit BleDatMngrSignal_PlotYawRateUpdate(); /*Move plotting to MainWindow process*/
        emit BleDatMngrSignal_PlotSpdUpdate(); /*Move plotting to MainWindow process*/
        emit BleDatMngrSignal_PlotPosErrUpdate();
        emit BleDatMngrSignal_PlotPidRegValUpdate();
    }


    if( ((uint8_t)(PrevSyncId+1)) != _inputSyncId)
    {
        QString SyncErrorString = QString("!!!Synchronization Error!!! BaseDataHandler SyncId: %1 |PrSyncId: %2").arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);
        emit BleDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BleDatMngrSignal_DebugTable_ScrollToBottom();
    }

    PrevSyncId = _inputSyncId;
}


void BleDataManager::BleDatMngr_ErrorWeigthDataHandler(char *data,uint32_t Size)
{

    static uint32_t PrevSyncId = 255U;

    uint8_t _inputSyncId = ((uint8_t)data[1]) ;
    static volatile uint8_t ExpectingFrameNumber= 0;


    static float IncomingErrorWeigthData[12];


    IncomingErrorWeigthData[0] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    IncomingErrorWeigthData[1] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6]));
    IncomingErrorWeigthData[2] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[10]));
    IncomingErrorWeigthData[3] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[14]));
    IncomingErrorWeigthData[4] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[18]));
    IncomingErrorWeigthData[5] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[22]));
    IncomingErrorWeigthData[6] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[26]));
    IncomingErrorWeigthData[7] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[30]));
    IncomingErrorWeigthData[8] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[34]));
    IncomingErrorWeigthData[9] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[38]));
    IncomingErrorWeigthData[10] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[42]));
    IncomingErrorWeigthData[11] = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[46]));

    emit BleDatMngrSignal_UpdateErrorWeigthData(IncomingErrorWeigthData[0],
                                                IncomingErrorWeigthData[1],
                                                IncomingErrorWeigthData[2],
                                                IncomingErrorWeigthData[3],
                                                IncomingErrorWeigthData[4],
                                                IncomingErrorWeigthData[5],
                                                IncomingErrorWeigthData[6],
                                                IncomingErrorWeigthData[7],
                                                IncomingErrorWeigthData[8],
                                                IncomingErrorWeigthData[9],
                                                IncomingErrorWeigthData[10],
                                                IncomingErrorWeigthData[11]);

    if(_inputSyncId != (uint8_t)(PrevSyncId+1))
    {

        QString SyncErrorString = QString("!!!Synchronization Error!!! ErrorWeigthDataHandler SyncId: %1 |PrSyncId: %2 ")
                                            .arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);

        emit BleDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        ExpectingFrameNumber = 0;
    }

    PrevSyncId = _inputSyncId;
}


void BleDataManager::BleDatMngr_DebugMessagerHandler(char *data,uint32_t size, BleDataManager::BLU_MessageID_t BLE_MessID)
{
    (void )BLE_MessID;
    (void)size;

    uint8_t _inputSyncId = data[1] ;
    static uint8_t PrevSyncId = 255;
    uint32_t ucTimeStamp;
    QString DebugString;
    ucTimeStamp = ConvToUint32(&data[2]);
    DebugString = QString::fromUtf8(&data[6]);
    emit BleDatMngrSignal_DebugTable_InsertDataRow(ucTimeStamp, BLE_MessID,_inputSyncId, DebugString);
    emit BleDatMngrSignal_DebugTable_ScrollToBottom();

    if(_inputSyncId != (uint8_t)(PrevSyncId+1))
    {
        QString SyncErrorString = QString("!!!Synchronization Error!!! DebugMessagerHandler SyncId: %2 |PrSyncId: %3 ")
                                      .arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);
        qDebug() << SyncErrorString;
        emit BleDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BleDatMngrSignal_DebugTable_ScrollToBottom();
    }

//    qDebug() << "SyncID:" << _inputSyncId << "Size:" << size << "Mes:" << DebugString;
//  qDebug() << "SyncID:" << (uint8_t)data[1] << "ucTimeSt:" << ucTimeStamp << "Mes:" << &data[6];

    PrevSyncId = _inputSyncId;
}



void BleDataManager::BleDatMngr_PidDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    float KpVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    float KiVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6]));
    float KdVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[10]));
    uint32_t ProbeTim = ConvToUint32(&data[14]);

    emit BleDatMngrSignal_UpdatePidData(KpVal,KiVal,KdVal,ProbeTim);
}

void BleDataManager::BleDatMngr_VehCfgDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    float ExpAvSpd                = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    uint32_t BlinkSt              = ConvToUint32(&data[6]);
    uint32_t TryDetEndLineMark    = ConvToUint32(&data[10]);

//    qDebug() << "BlinkSt:" << BlinkSt << "TryDetEndLineMark:" << TryDetEndLineMark;

    emit BleDatMngrSignal_UpdateVehCfgData(ExpAvSpd,BlinkSt,TryDetEndLineMark);
}

void BleDataManager::BleDatMngr_MotorsFactorsDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    uint32_t FacA_Lft    = ConvToUint32(&data[2]);
    uint32_t FacA_Rgt    = ConvToUint32(&data[6]);
    uint32_t FacB_Lft    = ConvToUint32(&data[10]);
    uint32_t FacB_Rht    = ConvToUint32(&data[14]);
    emit BleDatMngrSignal_UpdateMotorsFactors(FacA_Lft,FacA_Rgt,FacB_Lft,FacB_Rht);
}

void BleDataManager::BleDatMngr_EncodersCfgDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    float OneImpDist    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    float WheelBase    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6]));
    emit BleDatMngrSignal_UpdateEncoderCfgData(OneImpDist,WheelBase);
}

void BleDataManager::BleDatMngr_InputHanlder( char* data, uint32_t Size)
{
    static volatile BLU_MessageID_t BLE_MessageID;
    BLE_MessageID = ((BLU_MessageID_t)data[0] );

    switch(BLE_MessageID)
    {
        case BLU_MessageID_t::BLU_CommunicationStats:
        {

            BleDatMngr_CommunicationStatistics_Handler(data,Size);
            break;
        }

        case BLU_MessageID_t::BLU_BaseDataReport:
        {
            BleDatMngr_BaseDataHandler(data,Size);
            break;
        }

        case BLU_MessageID_t::BLU_NvM_ErrWeigthSensorData:

        {
            BleDatMngr_ErrorWeigthDataHandler(data,Size);
            break;
        }

        case BLU_NvM_VehCfgData:
        {
            BleDatMngr_VehCfgDataHandler(data,BLE_MessageID);
            break;
        }

        case BLU_NvM_LinePidRegData:
        {
            BleDatMngr_PidDataHandler(data,BLE_MessageID);
            break;
        }

        case BLU_NvM_MotorsFactorsData:
        {
            BleDatMngr_MotorsFactorsDataHandler(data,BLE_MessageID);
            break;
        }


        case BLU_NvM_EncoderModCfgData:
        {
            BleDatMngr_EncodersCfgDataHandler(data,BLE_MessageID);
            break;
        }


        case BLU_MessageID_t::BLU_DebugMessage:
        {
            BleDatMngr_DebugMessagerHandler(data,Size,BLE_MessageID);
            break;
        }


        default:
        {
            break;
        }
    }
}
