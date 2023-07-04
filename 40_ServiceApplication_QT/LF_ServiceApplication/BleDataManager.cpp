#include "BleDataManager.h"
#include "qelapsedtimer.h"
#include "qthread.h"

BleDataManager::BleDataManager()
{
    this->moveToThread(&BLE_Thread);

    connect(&bleConnection, SIGNAL(newData(QByteArray)), this, SLOT(BleDatMngr_InputHanlder(QByteArray)));




    BLE_Thread.start();
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

    qDebug() << "StatisticHandler SyncId" <<  ((uint8_t)value.at(1)) ;

    StatisticData.SyncId = ((uint8_t)value.at(1)) ;
    StatisticData.ucTimeStamp = ConvToUint32(value,2);
    StatisticData.RingBufferRemainingSize = ConvToUint16(value,6);
    StatisticData.RingBufferOverFlowCounter = ConvToUint16(value,8);
    StatisticData.TransmisstedMessagesCounter = ConvToUint16(value,10);
    StatisticData.RetransmissionCounter = ConvToUint16(value,12);


    if(PreviousSyncId != StatisticData.SyncId)
    {
        //        QString RgtWhlSpdSimu_s = QString::number(FullMapData.WhLftSp,'f',3);
        QString MyData = QString("RB_RemSize:%1 |OverFlCount:%2 |TrM_C: %3")
                             .arg(StatisticData.RingBufferRemainingSize).arg(StatisticData.RingBufferOverFlowCounter)
                             .arg(StatisticData.TransmisstedMessagesCounter);

//        ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
//        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(StatisticData.ucTimeStamp) ));
//        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(BleDataManager::BLE_CommunicationStatistics) ));
//        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
//                                    new QTableWidgetItem((QString)MyData.data() ) );

//        ui->statusbar->showMessage((QString)MyData.data(), 8000);
    }

    PreviousSyncId = StatisticData.SyncId;
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

            /*Copy full input MapData*/
            FullBaseData = IncomingLfBaseData;

            ExpectingFrameNumber = 0;
            FullFrameCounter++;


            /*TODO: Delegate this jobs to dedicated threads... (thread pools or something like it) */
            QElapsedTimer timer;

//            timer.start();
////            RefreshErrorIndicatorView();
//            qDebug() << "RefreshErrorIndicatorView TOOK: " << timer.elapsed() << "milliseconds";

//            timer.start();
////            BLE_UpdateDebugTableWithNewLfAppBaseData();
//            qDebug() << "BLE_UpdateDebugTableWithNewLfAppBaseData TOOK: " << timer.elapsed() << "milliseconds";

//            timer.start();
            MapPlot.LfGraph_AppendData(FullBaseData.CurrMapData.PosX,FullBaseData.CurrMapData.PosY);
            emit BleDatMngrSignal_MapPlotUpdate();

//            qDebug() << "MapGraph_LfGraph_UpdateReplot TOOK: " << timer.elapsed() << "milliseconds";

//            timer.start();

            YawRatePlot.LfGraph_AppendData((float)FullFrameCounter,FullBaseData.CurrMapData.YawRate);
            emit BleDatMngrSignal_YawRatePlotUpdate();
//            qDebug() << "YawRateGraph_LfGraph_UpdateReplot TOOK: " << timer.elapsed() << "milliseconds";


//            timer.start();
            SpdPlot.LfGraph_AppendData(FullFrameCounter,FullBaseData.CurrMapData.WhLftSp);
            emit BleDatMngrSignal_SpdPlotUpdate();
//            qDebug() << "SpdPlot_LfGraph_UpdateReplot TOOK: " << timer.elapsed() << "milliseconds";
        }
        else
        {
            if(PrevSyncId != _inputSyncId)
            {
                /*Ignore synchronization error detected*/
//                QString ExpectingFrameNumber_s = QString::number(ExpectingFrameNumber);
//                QString SyncErroMessage = QString("MapDatSyncErr cID:'%1'|pID:'%2'|mID:'%3' eeData:'%4' ")
//                                              .arg(_inputSyncId).arg(PrevSyncId).arg(BLE_MessID),arg(ExpectingFrameNumber_s);
//                ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
//                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,new QTableWidgetItem(SyncErroMessage));
//                ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 4) -> setData(Qt::BackgroundRole, QColor (250,0,0));
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



void BleDataManager::BleDatMngr_InputHanlder(const QByteArray &value)
{
    QElapsedTimer timer;
    timer.start();


    static volatile BLE_MessageID_t BLE_MessageID;
    BLE_MessageID = ((BLE_MessageID_t)value.at(0) );

    qDebug() <<  "NewFrameID:"  << "SyncID:" << ((uint8_t)value.at(1) );
    qDebug() << "BLE_inputDataHandler CurrThread:" << QThread::currentThread();


    switch(BLE_MessageID)
    {
        case BLE_MessageID_t::BLE_CommunicationStatistics:
        {

            BleDatMngr_BaseDataHandler(value,BLE_MessageID);
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

        default:
        {
            break;
        }
    }


    static uint32_t ShifterDelay = 0;


//    if( (ShifterDelay % 3) == 0)
//    {
////        ui->tableWidge_2->scrollToBottom();
//    }
    qDebug() << " Wrapper execution time TOOK: " << timer.elapsed() << "milliseconds";

}
