#include "bluetoothDataManager.h"
#include "qthread.h"

BluDataManager::BluDataManager()
{
    this->moveToThread(&BluDatMngr_Thread);
    bleutoothClassicConnection.moveToThread((&BluDatMngr_Thread)) ;

    connect(&bleutoothClassicConnection, SIGNAL(bluetoothSignalNewDataReceived(char *, uint32_t)), this, SLOT(BluDatMngr_InputHanlder( char*, uint32_t) ));

    BluDatMngr_Thread.start();
}

BluDataManager::~BluDataManager()
{
    BluDatMngr_Thread.terminate();
}

/*****************************************/
/*Tools functions start for Main Window and BLU*/
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


void BluDataManager::BluDatMngr_CommunicationStatistics_Handler(char *data,uint32_t Size)
{
    (void)Size;
    static BLU_StatisticData_t StatisticData = {0};
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
        emit BluDatMngrSignal_CommunicationStatisticsUpdate(StatisticData.ucTimeStamp,StatisticData.RingBufferRemainingSize,StatisticData.RingBufferOverFlowCounter,
                                                            StatisticData.TransmisstedMessagesCounter,StatisticData.RetransmissionCounter);
    }

    PreviousSyncId = StatisticData.SyncId;
}



void BluDataManager::BluDatMngr_BaseDataInsertToDebugTable(uint32_t FrameCounter)
{

        QString RgtWhlSpdSimu_s = QString::number(BluDataManager::FullBaseData.CurrMapData.WhLftSp,'f',3);
        QString LftWhlSpdSimu_s = QString::number(BluDataManager::FullBaseData.CurrMapData.WhRhtSp,'f',3);

        QString BaseMapData = QString("MapDat: pX: %1 |pY: %2 pO: %3 |L_sp: %4 |R_Sp: %5| Yr: %6")
                                  .arg(BluDataManager::FullBaseData.CurrMapData.PosX)
                                  .arg(BluDataManager::FullBaseData.CurrMapData.PosY)
                                  .arg(BluDataManager::FullBaseData.CurrMapData.PosO)
                                  .arg(LftWhlSpdSimu_s).arg(RgtWhlSpdSimu_s)
                                  .arg(BluDataManager::FullBaseData.CurrMapData.YawRate) ;

        QString BaseSensorDataP1 = QString("SenDat1: S0: %1 |S1: %2 |S2: %3 |S3: %4 |S4: %5 |S5: %6 |Err: %7")
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[0])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[1])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[2])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[3])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[4])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[5])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.PosError);

        QString BaseSensorDataP2 = QString("SenDat2: S6: %1 |S7: %2 |S8: %3 |S9: %4 |S10: %5 |S11: %6 |Err: %7")
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[6])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[7])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[8])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[9])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[10])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.SensorData[11])
                                       .arg(BluDataManager::FullBaseData.CurrSensorData.PosError);

//        QColor RowColor = QColor(255,255,255); /*Default row color (not modify color*/
        emit BluDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BluDataManager::FullBaseData.SyncId,BaseMapData);
        emit BluDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BluDataManager::FullBaseData.SyncId,BaseSensorDataP1);
        emit BluDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BluDataManager::FullBaseData.SyncId,BaseSensorDataP2);

        FrameCounter++;
}


void BluDataManager::BluDatMngr_BaseDataHandler(char *data,uint32_t Size)
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


    emit BluDatMngrSignal_UpdateOrientation(FullBaseData.CurrMapData.PosO);

    if(true == DebugTable_BaseDataLoggingState)
    {
    BluDatMngr_BaseDataInsertToDebugTable(FullFrameCounter);
    }

    emit BluDatMngrSignal_RefreshErrorIndicatorView( FullBaseData.CurrSensorData.SensorData[0],
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

    emit BluDatMngrSignal_PlotMapAppendData(FullBaseData.CurrMapData.PosX,FullBaseData.CurrMapData.PosY);
    emit BluDatMngrSignal_PlotYawRateAppendData(FullFrameCounter,FullBaseData.CurrMapData.YawRate);
    emit BluDatMngrSignal_PlotSpdAppendData(FullFrameCounter,FullBaseData.CurrMapData.WhLftSp,FullBaseData.CurrMapData.WhRhtSp);
    emit BluDatMngrSignal_PlotPosErrAppendData(FullFrameCounter,FullBaseData.CurrSensorData.PosError);
    emit BluDatMngrSignal_PlotPidRegValAppendData(FullFrameCounter,FullBaseData.LinePidRegData.PidRegCorrValue);
    emit BluDatMngrSignal_PlotOrientationAppendData(FullFrameCounter, FullBaseData.CurrMapData.PosO);
    emit BluDatMngrSignal_PlotTrvDistanceAppendData(FullFrameCounter, FullBaseData.CurrMapData.TravelledDistance);
    emit BluDatMngrSignal_PlotPosConfidenceAppendData(FullFrameCounter,
                                                      FullBaseData.CurrSensorData.LastLeftLinePosConfidence,
                                                      FullBaseData.CurrSensorData.LastRightLinePosConfidence);


    if( (false == MapPlotPlottingState) && (false == YawRatePlotPlottingState) && (false == SpdPlotPlottingState)
        && (false == DebugTableScrollingBottomIsActivState) && (false == PosErrPlotPlottingState) && (false == PidRegValPlotPlottingState)
        && (false == LinePosConfPlotPlottingState) && (false == TrvDistancePlotPlottingState) && (false == OrientationPlotPlottingState) )
    {
        PlottingInfoMutex.lock();
        MapPlotPlottingState = true;
        YawRatePlotPlottingState = true;
        SpdPlotPlottingState = true;
        PosErrPlotPlottingState = true;
        PidRegValPlotPlottingState = true;
        LinePosConfPlotPlottingState = true;
        TrvDistancePlotPlottingState = true;
        OrientationPlotPlottingState = true;

        PlottingInfoMutex.unlock();


        if(true == DebugTable_BaseDataLoggingState)
        {
            DebugTableScrollingBottonMutex.lock();
            emit BluDatMngrSignal_DebugTable_ScrollToBottom();
            DebugTableScrollingBottonMutex.unlock();
        }

        emit BluDatMngrSignal_PlotMapUpdate(); /*Move plotting to MainWindow process*/
        emit BluDatMngrSignal_PlotYawRateUpdate(); /*Move plotting to MainWindow process*/
        emit BluDatMngrSignal_PlotSpdUpdate(); /*Move plotting to MainWindow process*/
        emit BluDatMngrSignal_PlotPosErrUpdate();
        emit BluDatMngrSignal_PlotPidRegValUpdate();
        emit BluDatMngrSignal_PlotOrientationReplot();
        emit BluDatMngrSignal_PlotTrvDistanceReplot();
        emit BluDatMngrSignal_PlotPosConfidenceReplot();

    }


    if( ((uint8_t)(PrevSyncId+1)) != _inputSyncId)
    {
        QString SyncErrorString = QString("!!!Synchronization Error!!! BaseDataHandler SyncId: %1 |PrSyncId: %2").arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);
        emit BluDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BluDatMngrSignal_DebugTable_ScrollToBottom();
    }

    PrevSyncId = _inputSyncId;
}


void BluDataManager::BluDatMngr_ErrorWeigthDataHandler(char *data,uint32_t Size)
{
    (void)Size;

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

    emit BluDatMngrSignal_UpdateErrorWeigthData(IncomingErrorWeigthData[0],
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

        emit BluDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        ExpectingFrameNumber = 0;
    }

    PrevSyncId = _inputSyncId;
}


void BluDataManager::BluDatMngr_DebugMessagerHandler(char *data,uint32_t size, BLU_MessageID_t BLE_MessID)
{
    (void )BLE_MessID;
    (void)size;

    uint8_t _inputSyncId = data[1] ;
    static uint8_t PrevSyncId = 255;
    uint32_t ucTimeStamp;
    QString DebugString;
    ucTimeStamp = ConvToUint32(&data[2]);
    DebugString = QString::fromUtf8(&data[6]);
    emit BluDatMngrSignal_DebugTable_InsertDataRow(ucTimeStamp, BLE_MessID,_inputSyncId, DebugString);
    emit BluDatMngrSignal_DebugTable_ScrollToBottom();

    if(_inputSyncId != (uint8_t)(PrevSyncId+1))
    {
        QString SyncErrorString = QString("!!!Synchronization Error!!! DebugMessagerHandler SyncId: %2 |PrSyncId: %3 ")
                                      .arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);
        qDebug() << SyncErrorString;
        emit BluDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BluDatMngrSignal_DebugTable_ScrollToBottom();
    }

//    qDebug() << "SyncID:" << _inputSyncId << "Size:" << size << "Mes:" << DebugString;
//  qDebug() << "SyncID:" << (uint8_t)data[1] << "ucTimeSt:" << ucTimeStamp << "Mes:" << &data[6];

    PrevSyncId = _inputSyncId;
}



void BluDataManager::BluDatMngr_PidDataHandler(char* data, uint32_t Size)
{
    (void)Size;
    float KpVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    float KiVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6]));
    float KdVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[10]));
    uint32_t ProbeTim = ConvToUint32(&data[14]);

    emit BluDatMngrSignal_UpdatePidData(KpVal,KiVal,KdVal,ProbeTim);
}

void BluDataManager::BluDatMngr_RightAngleDataHandler(char* data, uint32_t Size)
{
    (void)Size;
    float rAgPidKp               = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    float rAgPidKd               = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6]));
    float rAgBaseSpd             = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[10]));
    float rAgMaxYawRate          = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[14]));
    float rAgBrakeSpeedTh        = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[18]));
    float rAgBrakingTime         = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[22]));
    float rAgOriChange           = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[26]));
    float rAgOriChangeAfterBrake = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[30]));
    uint32_t rAgProbeTime        =                              ConvToUint32(&data[34]);

    emit BluDatMngrSignal_UpdateRgAngleHndlrData(rAgPidKp,rAgPidKd,
                                                 rAgBaseSpd,rAgMaxYawRate,
                                                 rAgBrakeSpeedTh, rAgBrakingTime,
                                                 rAgOriChange, rAgOriChangeAfterBrake,
                                                 rAgProbeTime);
}

void BluDataManager::BluDatMngr_VehCfgDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    float ExpAvSpd                = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    uint32_t BlinkSt              = ConvToUint32(&data[6]);
    uint32_t TryDetEndLineMark    = ConvToUint32(&data[10]);
    uint32_t IrSensorIsEnabled    = ConvToUint32(&data[14]);

//    qDebug() << "BlinkSt:" << BlinkSt << "TryDetEndLineMark:" << TryDetEndLineMark;

    emit BluDatMngrSignal_UpdateVehCfgData(ExpAvSpd,BlinkSt,TryDetEndLineMark,IrSensorIsEnabled);
}

void BluDataManager::BluDatMngr_MotorsFactorsDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    uint32_t FacA_Lft    = ConvToUint32(&data[2]);
    uint32_t FacA_Rgt    = ConvToUint32(&data[6]);
    uint32_t FacB_Lft    = ConvToUint32(&data[10]);
    uint32_t FacB_Rht    = ConvToUint32(&data[14]);
    emit BluDatMngrSignal_UpdateMotorsFactors(FacA_Lft,FacA_Rgt,FacB_Lft,FacB_Rht);
}

void BluDataManager::BluDatMngr_EncodersCfgDataHandler( char* data, uint32_t Size)
{
    (void)Size;
    float OneImpDist    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[2]));
    float WheelBase    = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6]));
    emit BluDatMngrSignal_UpdateEncoderCfgData(OneImpDist,WheelBase);
}

void BluDataManager::BluDatMngr_SpeedProfileHandler( char* data, uint32_t Size)
{
    (void)Size;
    BLU_NvM_SpdProfileData_t SpdProfileData;
    memcpy(&SpdProfileData,&data[2],sizeof(BLU_NvM_SpdProfileData_t) );
    emit BluDatMngrSignal_UpdateSpeedProfileData(SpdProfileData);
}

void BluDataManager::BluDatMngr_InputHanlder( char* data, uint32_t Size)
{
    static volatile BLU_MessageID_t BLU_MessageID;
    BLU_MessageID = ((BLU_MessageID_t)data[0] );

    switch(BLU_MessageID)
    {
        case BLU_MessageID_t::BLU_CommunicationStats:
        {

            BluDatMngr_CommunicationStatistics_Handler(data,Size);
            break;
        }

        case BLU_MessageID_t::BLU_BaseDataReport:
        {
            BluDatMngr_BaseDataHandler(data,Size);
            break;
        }

        case BLU_MessageID_t::BLU_NvM_ErrWeigthSensorData:

        {
            BluDatMngr_ErrorWeigthDataHandler(data,Size);
            break;
        }

        case BLU_NvM_VehCfgData:
        {
            BluDatMngr_VehCfgDataHandler(data,BLU_MessageID);
            break;
        }

        case BLU_NvM_LinePidRegData:
        {
            BluDatMngr_PidDataHandler(data,BLU_MessageID);
            break;
        }
        case BLU_NvM_RightAgHndlrData:
        {
            BluDatMngr_RightAngleDataHandler(data,BLU_MessageID);
            break;
        }
        case BLU_NvM_MotorsFactorsData:
        {
            BluDatMngr_MotorsFactorsDataHandler(data,BLU_MessageID);
            break;
        }


        case BLU_NvM_EncoderModCfgData:
        {
            BluDatMngr_EncodersCfgDataHandler(data,BLU_MessageID);
            break;
        }

        case BLU_MessageID_t::BLU_NvM_SpdProfileData:
        {
            BluDatMngr_SpeedProfileHandler(data,BLU_MessageID);
            break;
        }

        case BLU_MessageID_t::BLU_DebugMessage:
        {
            BluDatMngr_DebugMessagerHandler(data,Size,BLU_MessageID);
            break;
        }



        default:
        {
            break;
        }
    }
}
