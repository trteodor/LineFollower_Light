#include "BleDataManager.h"
#include "qthread.h"

BleDataManager::BleDataManager()
{
    this->moveToThread(&BleDatMngr_Thread);
    bleConnection.moveToThread((&BleDatMngr_Thread)) ;

    connect(&bleConnection, SIGNAL(newData(QByteArray)), this, SLOT(BleDatMngr_InputHanlder(QByteArray)));

    BleDatMngr_Thread.start();
}


/*****************************************/
/*Tools functions start for Main Window and BLE*/
/**/
uint32_t ConvToUint32(const QByteArray &value, uint8_t ShiftValue)
{
    return (((uint8_t)value.at(ShiftValue) ) | ( (uint8_t)value.at(ShiftValue+1) << 8) \
            | ( (uint8_t)value.at(ShiftValue+2) << 16) | ( (uint8_t)value.at(ShiftValue+3) << 24));

}
/**************************************************************/
uint32_t ConvToUint16(const QByteArray &value, uint8_t ShiftValue)
{
    return (((uint8_t)value.at(ShiftValue) ) | ( (uint8_t)value.at(ShiftValue+1) << 8) );
}
/**************************************************************/
float ieee_uint32_AsBitsTo_float32(uint32_t f)
{
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}
/*Tools functions end */


void BleDataManager::BleDatMngr_CommunicationStatistics_Handler(const QByteArray &value)
{
    static BleDataManager::BLE_StatisticData_t StatisticData = {0};
    static uint8_t PreviousSyncId = 255U;

//    qDebug() << "StatisticHandler SyncId" <<  ((uint8_t)value.at(1)) ;

    StatisticData.SyncId = ((uint8_t)value.at(1)) ;
    StatisticData.ucTimeStamp = ConvToUint32(value,2);
    StatisticData.RingBufferRemainingSize = ConvToUint16(value,6);
    StatisticData.RingBufferOverFlowCounter = ConvToUint16(value,8);
    StatisticData.TransmisstedMessagesCounter = ConvToUint16(value,10);
    StatisticData.RetransmissionCounter = ConvToUint16(value,12);


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
        QString BaseMapData = QString("MapDat: pX: %1 |pY: %2 |L_sp: %3 |R_Sp: %4| Yr: %5")
                                  .arg(BleDataManager::FullBaseData.CurrMapData.PosX).arg(BleDataManager::FullBaseData.CurrMapData.PosY)
                                  .arg(LftWhlSpdSimu_s).arg(RgtWhlSpdSimu_s).arg(BleDataManager::FullBaseData.CurrMapData.YawRate) ;

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


void BleDataManager::BleDatMngr_BaseDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID)
{
//    qDebug() << "ok iam here";

    static uint32_t PrevSyncId = 0U;
    static uint32_t FullFrameCounter = 0;
    uint8_t _inputSyncId = ((uint8_t)value.at(1)) ;
    static volatile uint8_t ExpectingFrameNumber= 0;


    static BleDataManager::BLE_LfDataReport_t IncomingLfBaseData;


    if( (0 == ExpectingFrameNumber) && (BLE_MessID == BleDataManager::BLE_BaseDataReport_part1)  && (PrevSyncId != _inputSyncId))
    {
        IncomingLfBaseData.SyncId = _inputSyncId;
        IncomingLfBaseData.ucTimeStamp = ConvToUint32(value,2);
        IncomingLfBaseData.CurrMapData.WhLftSp = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6)) ;
        IncomingLfBaseData.CurrMapData.WhRhtSp =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10)) ;
        IncomingLfBaseData.CurrMapData.YawRate =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,14)) ;
        ExpectingFrameNumber = 1;
    }
    else if( (1 == ExpectingFrameNumber) && (BLE_MessID == BleDataManager::BLE_BaseDataReport_part2)  && (PrevSyncId == _inputSyncId))
    {
        IncomingLfBaseData.CurrMapData.PosX = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2)) ;
        IncomingLfBaseData.CurrMapData.PosY =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6)) ;
        IncomingLfBaseData.CurrMapData.TravelledDistance =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10)) ;

        IncomingLfBaseData.CurrSensorData.SensorData[0] =((uint8_t)value.at(14)) ;
        IncomingLfBaseData.CurrSensorData.SensorData[1] =((uint8_t)value.at(15)) ;
        IncomingLfBaseData.CurrSensorData.SensorData[2] =((uint8_t)value.at(16)) ;
        IncomingLfBaseData.CurrSensorData.SensorData[3] =((uint8_t)value.at(17)) ;
        IncomingLfBaseData.CurrSensorData.SensorData[4] =((uint8_t)value.at(18)) ;
        IncomingLfBaseData.CurrSensorData.SensorData[5] =((uint8_t)value.at(19)) ;

        IncomingLfBaseData.CurrSensorData.SensorData[5] =((uint8_t)value.at(19)) ;
        ExpectingFrameNumber = 2;
    }
    else
    {
        if( (2 == ExpectingFrameNumber)  && (BLE_MessID == BleDataManager::BLE_MessageID_t::BLE_BaseDataReport_part3) && (PrevSyncId == _inputSyncId) )
        {
            IncomingLfBaseData.CurrSensorData.SensorData[6]  =((uint8_t)value.at(2)) ;
            IncomingLfBaseData.CurrSensorData.SensorData[7]  =((uint8_t)value.at(3)) ;
            IncomingLfBaseData.CurrSensorData.SensorData[8]  =((uint8_t)value.at(4)) ;
            IncomingLfBaseData.CurrSensorData.SensorData[9]  =((uint8_t)value.at(5)) ;
            IncomingLfBaseData.CurrSensorData.SensorData[10] =((uint8_t)value.at(6)) ;
            IncomingLfBaseData.CurrSensorData.SensorData[11] =((uint8_t)value.at(7)) ;

            IncomingLfBaseData.CurrSensorData.PosError =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,8));

            IncomingLfBaseData.CurrSensorData.LastLeftLinePosConfidence =((uint8_t)value.at(12)) ;
            IncomingLfBaseData.CurrSensorData.LastRightLinePosConfidence =((uint8_t)value.at(13)) ;

            IncomingLfBaseData.CurrPidRegData.PidRegCorrValue = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,16));


            /*Copy full input MapData*/
            FullBaseData = IncomingLfBaseData;

            ExpectingFrameNumber = 0;
            FullFrameCounter++;



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
            emit BleDatMngrSignal_PlotSpdAppendData(FullFrameCounter,FullBaseData.CurrMapData.WhLftSp);
            emit BleDatMngrSignal_PlotPosErrAppendData(FullFrameCounter,FullBaseData.CurrSensorData.PosError);
            emit BleDatMngrSignal_PlotPidRegValAppendData(FullFrameCounter,IncomingLfBaseData.CurrPidRegData.PidRegCorrValue);


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
            else
            {
                //qDebug() << "Plotting not finished skip";
                /*Refresh plots in next frame*/
                /*Nothing to do*/
            }


        }
        else
        {
            if(PrevSyncId != _inputSyncId)
            {

                QString SyncErrorString = QString("!!!Synchronization Error!!! SyncId: %1 |PrSyncId: %2").arg(_inputSyncId).arg(PrevSyncId) ;
                QColor RowColor = QColor(255,0,0);

                emit BleDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
                ExpectingFrameNumber = 0;
            }
            else
            {
                ; /*Frame already handled ignore*/
            }

        }
    }

    PrevSyncId = _inputSyncId;
}


void BleDataManager::BleDatMngr_ErrorWeigthDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID)
{
    //    qDebug() << "ok iam here";

    static uint32_t PrevSyncId = 0U;


    uint8_t _inputSyncId = ((uint8_t)value.at(1)) ;
    static volatile uint8_t ExpectingFrameNumber= 0;


    static float IncomingErrorWeigthData[12];


    if( (0 == ExpectingFrameNumber) && (BLE_MessID == BleDataManager::BLE_NvM_ErrWeigthSensorData_part1)  && (PrevSyncId != _inputSyncId))
    {

        IncomingErrorWeigthData[0] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2));
        IncomingErrorWeigthData[1] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6));
        IncomingErrorWeigthData[2] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10));
        IncomingErrorWeigthData[3] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,14));

        ExpectingFrameNumber = 1;
    }
    else if( (1 == ExpectingFrameNumber) && (BLE_MessID == BleDataManager::BLE_NvM_ErrWeigthSensorData_part2)  && (PrevSyncId == _inputSyncId))
    {
        IncomingErrorWeigthData[4] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2));
        IncomingErrorWeigthData[5] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6));
        IncomingErrorWeigthData[6] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10));
        IncomingErrorWeigthData[7] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,14));
        ExpectingFrameNumber = 2;
    }
    else
    {
        if( (2 == ExpectingFrameNumber)  && (BLE_MessID == BleDataManager::BLE_MessageID_t::BLE_NvM_ErrWeigthSensorData_part3) && (PrevSyncId == _inputSyncId) )
        {
            IncomingErrorWeigthData[8] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2));
            IncomingErrorWeigthData[9] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6));
            IncomingErrorWeigthData[10] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10));
            IncomingErrorWeigthData[11] = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,14));

            ExpectingFrameNumber = 0;
//            qDebug() << "BleDatMngr_ErrorWeigthDataHandler Received full frame!";

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
                                                        IncomingErrorWeigthData[11]  );

        }
        else
        {
            if(PrevSyncId != _inputSyncId)
            {

                QString SyncErrorString = QString("!!!Synchronization Error!!! ErrorWeigthDataHandler SyncId: %1 |PrSyncId: %2 ")
                                                    .arg(_inputSyncId).arg(PrevSyncId) ;
                QColor RowColor = QColor(255,0,0);

                emit BleDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
                ExpectingFrameNumber = 0;
            }
            else
            {
                ; /*Frame already handled ignore*/
            }

        }
    }

    PrevSyncId = _inputSyncId;
}


void BleDataManager::BleDatMngr_DebugMessagerHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID)
{
    (void )BLE_MessID;


    /*
     * Frist Frame:
     * |FRAMEID|SYNCID|REMAINING_FRAMES| ucTIME | DATA 13bytes
     * Other frames:
     * |FRAME_ID|SYNC_ID|REMAINING_FRAMES | DATA 17bytes
     *
     * If remaining frames is equal "0" then desktop software
     * should tract it as a last frame
     *
     * REMAINING_FRAMES counter shall be decrement with next frames
     *
     */

    static uint32_t PrevSyncId = 255U;


    uint8_t _inputSyncId = ((uint8_t)value.at(1)) ;

    static uint8_t RemainingFramesCounter = 0;
    static uint32_t ucTimeStamp;
    static QString DebugString;


    if( PrevSyncId != _inputSyncId && (RemainingFramesCounter == 0) )
    {
        /*Frist message frame and any synchronization error isn't detected*/
        RemainingFramesCounter = value.at(2);
        ucTimeStamp = ConvToUint32(value,3);
        DebugString = QString::fromUtf8(value.toStdString());

        DebugString.remove(0,7);
        DebugString.remove(QChar::Null);
        if(RemainingFramesCounter == 0)
        {
            /*Last message frame detected!*/
            emit BleDatMngrSignal_DebugTable_InsertDataRow(ucTimeStamp, BLE_MessID,_inputSyncId, DebugString);
            emit BleDatMngrSignal_DebugTable_ScrollToBottom();
        }
    }
    else if(PrevSyncId == _inputSyncId && (RemainingFramesCounter -1) == value.at(2) && ( (RemainingFramesCounter -1) == 0 ) )
    {
        /*Last message frame detected!*/
        RemainingFramesCounter = value.at(2);
        QString DbgMessageStringPart = QString::fromUtf8(value.toStdString());
        DbgMessageStringPart.remove(0,3);
        DbgMessageStringPart.remove(QChar::Null);
        DebugString.append(DbgMessageStringPart);

        emit BleDatMngrSignal_DebugTable_InsertDataRow(ucTimeStamp, BLE_MessID,_inputSyncId, DebugString);
        emit BleDatMngrSignal_DebugTable_ScrollToBottom();

    }
    else if(PrevSyncId == _inputSyncId && (RemainingFramesCounter -1) == value.at(2))
    {
        /*new message frame detected*/
        RemainingFramesCounter = value.at(2);
        QString DbgMessageStringPart = QString::fromUtf8(value.toStdString());
        DbgMessageStringPart.remove(0,3);
        DbgMessageStringPart.remove(QChar::Null);
        DebugString.append(DbgMessageStringPart);
    }
    else
    {
        RemainingFramesCounter = 0;

        QString SyncErrorString = QString("!!!Synch Error!!! DbgMsgHandler SyncId: %1 |PrSyncId: %2 ")
                                      .arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);

        emit BleDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BleDatMngrSignal_DebugTable_ScrollToBottom();
    }

//     qDebug() << "SyncID:" << _inputSyncId << "ucTimeSt:" << ucTimeStamp << "Mes:" << DebugString;

    PrevSyncId = _inputSyncId;
}



void BleDataManager::BleDatMngr_PidDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID)
{
    (void)BLE_MessID;
    float KpVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2));
    float KiVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6));
    float KdVal    = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10));
    float ProbeTim = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,14));

    emit BleDatMngrSignal_UpdatePidData(KpVal,KiVal,KdVal,ProbeTim);
}

void BleDataManager::BleDatMngr_VehCfgDataHandler(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID)
{
    (void)BLE_MessID;
    float ExpAvSpd                = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2));
    uint32_t BlinkSt              = ConvToUint32(value,6);
    uint32_t TryDetEndLineMark    = ConvToUint32(value,10);

    emit BleDatMngrSignal_UpdateVehCfgData(ExpAvSpd,BlinkSt,TryDetEndLineMark);
}



void BleDataManager::BleDatMngr_InputHanlder(const QByteArray &value)
{

    static volatile BLE_MessageID_t BLE_MessageID;
    BLE_MessageID = ((BLE_MessageID_t)value.at(0) );

    switch(BLE_MessageID)
    {
        case BLE_MessageID_t::BLE_CommunicationStatistics:
        {

            BleDatMngr_CommunicationStatistics_Handler(value);
            break;
        }

        case BLE_MessageID_t::BLE_BaseDataReport_part1:
        {
            BleDatMngr_BaseDataHandler(value,BLE_MessageID);
            break;
        }
        case BLE_MessageID_t::BLE_BaseDataReport_part2:
        {
            BleDatMngr_BaseDataHandler(value,BLE_MessageID);
            break;
        }
        case BLE_MessageID_t::BLE_BaseDataReport_part3:
        {
            BleDatMngr_BaseDataHandler(value,BLE_MessageID);
            break;
        }

        case BLE_MessageID_t::BLE_NvM_ErrWeigthSensorData_part1:
        case BLE_MessageID_t::BLE_NvM_ErrWeigthSensorData_part2:
        case BLE_MessageID_t::BLE_NvM_ErrWeigthSensorData_part3:
        {

            BleDatMngr_ErrorWeigthDataHandler(value,BLE_MessageID);
            break;
        }

        case BLE_NvM_VehCfgData:
        {
            BleDatMngr_VehCfgDataHandler(value,BLE_MessageID);
            break;
        }

        case BLE_NvM_PidRegData:
        {
            BleDatMngr_PidDataHandler(value,BLE_MessageID);
            break;
        }

        case BLE_MessageID_t::BLE_DebugMessage:
        {
            BleDatMngr_DebugMessagerHandler(value,BLE_MessageID);
            break;
        }


        default:
        {
            break;
        }
    }
}
