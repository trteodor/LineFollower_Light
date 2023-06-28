#include "mainwindow.h"
#include "ui_mainwindow.h"

/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    BLE_InitializeQTConnections();


    SpdPlot = new LF_ServiceAppPlot(ui->SpdPlotW,QCPGraph::lsLine) ;

    YawRatePlot = new LF_ServiceAppPlot(ui->YawRatePlotW,QCPGraph::lsLine);

    MapPlot = new LF_ServiceAppPlot(ui->MapViewWidget,QCPGraph::lsNone);




    QFuture<void> f1 = QtConcurrent::run(&(MapPlot->LfGraph_AppendData(10,10)  ) ) ;
    f1.waitForFinished();


    /*Debug Table configure*/
    ui->tableWidge_2->setRowCount(1);
    ui->tableWidge_2->setColumnWidth(0,50);
    ui->tableWidge_2->setColumnWidth(1,50);
    ui->tableWidge_2->setColumnWidth(2,28);
    ui->tableWidge_2->setColumnWidth(3,10);
    ui->tableWidge_2->setColumnWidth(4,400);

    ui->tableWidge_2->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);

    QFile f(":qdarkstyle/dark/darkstyle.qss");

    if (!f.exists())   {
        qDebug() << "Unable to set stylesheet, file not found\n";
    }

    else   {
        f.open(QFile::ReadOnly | QFile::Text);
        QTextStream ts(&f);
        qApp->setStyleSheet(ts.readAll());
    }

}
/*********************************************************************************************************/
MainWindow::~MainWindow()
{
    on_BLE_SuspendFakeProdButton_clicked();
    QThread::msleep(25);
    on_BLE_SuspendFakeProdButton_clicked();
    QThread::msleep(25);
    on_BLE_SuspendFakeProdButton_clicked();
    QThread::msleep(25);
    /*Send the command 3x to be sure that fakeProducer will be stopped, if not then re-connection may be impossible
     * - HW reset may be required
    */

    emit BLE_DisconnectDevice();

    delete SpdPlot;
    delete YawRatePlot;
    delete MapPlot;


    delete ui;

    qDebug("Reached end");
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



/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/

void MainWindow::on_GeneralPlotDataClear_pb_clicked()
{

}


/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
void MainWindow::BLE_InitializeQTConnections(void)
{
    /*BLE Signals*/
    connect(this, SIGNAL(BLE_BlockData(bool)),&bleConnection,SLOT(BlockData(bool)));
    /* Signal and Slots */
    /* Search Button */
    connect(ui->BLE_ScanButton, SIGNAL(clicked()),&bleConnection, SLOT(startScan()));

    /* Connect Button */
    connect(ui->BLE_ConnectButton,SIGNAL(clicked()), this, SLOT(BLE_connectDevice()));
    connect(this, SIGNAL(BLE_DisconnectDevice() ),&bleConnection,SLOT(DisconnectDevice() ));
    //    /* Bleutooth States */
    connect(&bleConnection, SIGNAL(changedState(bluetoothleUART::bluetoothleState)),this,SLOT(BLE_changedState(bluetoothleUART::bluetoothleState)));
}
/*********************************************************************************************************/
void MainWindow::BLE_changedState(bluetoothleUART::bluetoothleState state){

    static uint8_t deviceCounter_l = 0;

    qDebug() << state;

    switch(state){

    case bluetoothleUART::Scanning:
    {
        ui->BLE_StatusLabel->setText("State:Scanning");
        ui->BLE_DetectedDeviceComboBox->clear();

        ui->BLE_ConnectButton->setEnabled(false);
        ui->BLE_ScanButton->setEnabled(false);
        ui->BLE_DetectedDeviceComboBox->setEnabled(true);

        ui->statusbar->showMessage("Searching for low energy devices...",1000);
        break;
    }

    case bluetoothleUART::NewDeviceDiscovered:
    {
        ui->BLE_StatusLabel->setText("State:NewDevDet");
        bleConnection.getDeviceList(FoundDevices);

        if(!FoundDevices.empty()){

            ui->BLE_DetectedDeviceComboBox->addItem(FoundDevices.at(deviceCounter_l));
            /* Initialise Slot startConnect(int) -> button press ui->BLE_ConnectButton */
            connect(this, SIGNAL(BLE_connectToDevice(int)),&bleConnection,SLOT(startConnect(int)));

            ui->BLE_ConnectButton->setEnabled(true);
            ui->BLE_ScanButton->setEnabled(true);
            ui->BLE_DetectedDeviceComboBox->setEnabled(true);

            QString AutoConnDevName = ui->BLE_AutoConnDevNameL->text();

            if( ui->BLE_DetectedDeviceComboBox->itemText(deviceCounter_l) ==  AutoConnDevName
                && ui->BLE_AutoConnCheckBox->isChecked() )
            {
                ui->BLE_DetectedDeviceComboBox->setCurrentIndex(deviceCounter_l);
                ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
                ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,new QTableWidgetItem(QString("Expected dev. name Matched: Autoconnection ") ));
                emit BLE_connectToDevice(deviceCounter_l);
            }
            else
            {
                ui->statusbar->showMessage("Please select BLE device",1000);
            }
            deviceCounter_l++;
        }

        break;
    }

    case bluetoothleUART::ScanFinished:
    {
        ui->BLE_StatusLabel->setText("State:ScanFinished");
    }
    case bluetoothleUART::Connecting:
    {
                ui->BLE_StatusLabel->setText("State:Connecting");
        ui->BLE_ConnectButton->setEnabled(false);
        ui->BLE_ScanButton->setEnabled(false);
//        ui->BLE_DetectedDeviceComboBox->setEnabled(false);

        ui->statusbar->showMessage("Connecting to device...",1000);
        break;
    }
    case bluetoothleUART::Connected:
    {
        ui->BLE_StatusLabel->setText("State:ConnNotReady");
        break;
    }
    case bluetoothleUART::Disconnected:
    {
        ui->BLE_StatusLabel->setText("State:Disconnected");
        ui->BLE_ConnectButton->setEnabled(true);
        ui->BLE_ScanButton->setEnabled(true);
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->BLE_StatusLabel->setAutoFillBackground(true);
        ui->BLE_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");


        deviceCounter_l = 0;
        break;
    }
    case bluetoothleUART::ServiceFound:
    {
        break;
    }
    case bluetoothleUART::AcquireData:
    {
        connect(&bleConnection, SIGNAL(newData(QByteArray)), this, SLOT(BLE_DataHandler(QByteArray)));
        ui->statusbar->showMessage("Aquire data",1000);

        ui->BLE_StatusLabel->setText("State:ConnReadyAcq");
        QVariant variant= QColor (220,255,220);
        QString colcode = variant.toString();
        ui->BLE_StatusLabel->setAutoFillBackground(true);
        ui->BLE_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");


        break;
    }
    default:
        break;


    }


}
/*********************************************************************************************************/
void MainWindow::BLE_CommunicationStatistics_Handler(const QByteArray &value)
{
    static MainWindow::BLE_StatisticData_t StatisticData = {0};
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

        ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
        ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,1,new QTableWidgetItem(QString::number(StatisticData.ucTimeStamp) ));
        ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,2,new QTableWidgetItem(QString::number(BLE_CommunicationStatistics) ));
        ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,
                                  new QTableWidgetItem((QString)MyData.data() ) );

        ui->statusbar->showMessage((QString)MyData.data(), 8000);
    }

    PreviousSyncId = StatisticData.SyncId;
}
/*********************************************************************************************************/
void MainWindow::RefreshErrorIndicatorView(void)
{
//    if(FullBaseData.CurrSensorData.SensorData[0] > 50)

    ui->S1_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[0]));
    ui->S2_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[1]));
    ui->S3_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[2]));
    ui->S4_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[3]));
    ui->S5_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[4]));
    ui->S6_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[5]));
    ui->S7_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[6]));
    ui->S8_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[7]));
    ui->S9_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[8]));
    ui->S10_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[9]));
    ui->S11_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[10]));
    ui->S12_label->setText(QString::number(FullBaseData.CurrSensorData.SensorData[11]));

    ui->PositionErrorLabel->setText(QString::number(FullBaseData.CurrSensorData.LastRightLinePosConfidence) );
    ui->PositionErrorLabel->setText(QString::number(FullBaseData.CurrSensorData.LastLeftLinePosConfidence) );
    ui->PositionErrorLabel->setText(QString::number(FullBaseData.CurrSensorData.PosError) );



    QVariant variant= QColor (255,0,0);
    QString colcode = variant.toString();
    ui->PositionErrorLabel->setAutoFillBackground(true);
    ui->PositionErrorLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

    QFont f( "Arial", 18, QFont::Bold);
    ui->PositionErrorLabel->setFont(f);



    if(FullBaseData.CurrSensorData.SensorData[0] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S1_label->setAutoFillBackground(true);
        ui->S1_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S1_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S1_label->setAutoFillBackground(true);
        ui->S1_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S1_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[1] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S2_label->setAutoFillBackground(true);
        ui->S2_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S2_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S2_label->setAutoFillBackground(true);
        ui->S2_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S2_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[2] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S3_label->setAutoFillBackground(true);
        ui->S3_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S3_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S3_label->setAutoFillBackground(true);
        ui->S3_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S3_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[3] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S4_label->setAutoFillBackground(true);
        ui->S4_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S4_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S4_label->setAutoFillBackground(true);
        ui->S4_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S4_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[4] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S5_label->setAutoFillBackground(true);
        ui->S5_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S5_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S5_label->setAutoFillBackground(true);
        ui->S5_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S5_label->setFont(f);
    }


    if(FullBaseData.CurrSensorData.SensorData[5] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S6_label->setAutoFillBackground(true);
        ui->S6_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S6_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S6_label->setAutoFillBackground(true);
        ui->S6_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S6_label->setFont(f);
    }


    if(FullBaseData.CurrSensorData.SensorData[6] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S7_label->setAutoFillBackground(true);
        ui->S7_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S7_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S7_label->setAutoFillBackground(true);
        ui->S7_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S7_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[7] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S8_label->setAutoFillBackground(true);
        ui->S8_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S8_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S8_label->setAutoFillBackground(true);
        ui->S8_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S8_label->setFont(f);
    }


    if(FullBaseData.CurrSensorData.SensorData[8] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S9_label->setAutoFillBackground(true);
        ui->S9_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S9_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S9_label->setAutoFillBackground(true);
        ui->S9_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S9_label->setFont(f);
    }


    if(FullBaseData.CurrSensorData.SensorData[9] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S10_label->setAutoFillBackground(true);
        ui->S10_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S10_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S10_label->setAutoFillBackground(true);
        ui->S10_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S10_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[10] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S11_label->setAutoFillBackground(true);
        ui->S11_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S11_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S11_label->setAutoFillBackground(true);
        ui->S11_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S11_label->setFont(f);
    }

    if(FullBaseData.CurrSensorData.SensorData[11] > 60)
    {
        QVariant variant= QColor (255,120,120);
        QString colcode = variant.toString();
        ui->S12_label->setAutoFillBackground(true);
        ui->S12_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S12_label->setFont(f);
    }
    else
    {
        QVariant variant= QColor (255,255,255);
        QString colcode = variant.toString();
        ui->S12_label->setAutoFillBackground(true);
        ui->S12_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

        QFont f( "Arial", 13, QFont::Bold);
        ui->S12_label->setFont(f);
    }

}
/*********************************************************************************************************/
void MainWindow::UpdateDebugTableWithNewLfAppBaseData(void)
{
    static uint16_t FrameCounter = 0U;

    QString RgtWhlSpdSimu_s = QString::number(FullBaseData.CurrMapData.WhLftSp,'f',3);
    QString LftWhlSpdSimu_s = QString::number(FullBaseData.CurrMapData.WhRhtSp,'f',3);
    QString BaseMapData = QString("pX: %1 |pY: %2 |L_sp: %3 |R_Sp: %4| Yr: %5")
                              .arg(FullBaseData.CurrMapData.PosX).arg(FullBaseData.CurrMapData.PosY)
                              .arg(LftWhlSpdSimu_s).arg(RgtWhlSpdSimu_s).arg(FullBaseData.CurrMapData.YawRate) ;

    ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,1,new QTableWidgetItem(QString::number(FullBaseData.ucTimeStamp) ));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,3,new QTableWidgetItem(QString::number(FullBaseData.SyncId)));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,
                              new QTableWidgetItem((QString)BaseMapData.data() ) );


    QString BaseSensorDataP1 = QString("S0: %1 |S1: %2 |S2: %3 |S3: %4 |S4: %5 |S5: %6 |Err: %7")
                                   .arg(FullBaseData.CurrSensorData.SensorData[0])
                                   .arg(FullBaseData.CurrSensorData.SensorData[1])
                                   .arg(FullBaseData.CurrSensorData.SensorData[2])
                                   .arg(FullBaseData.CurrSensorData.SensorData[3])
                                   .arg(FullBaseData.CurrSensorData.SensorData[4])
                                   .arg(FullBaseData.CurrSensorData.SensorData[5])
                                   .arg(FullBaseData.CurrSensorData.PosError);

    QString BaseSensorDataP2 = QString("S6: %1 |S7: %2 |S8: %3 |S9: %4 |S10: %5 |S11: %6 |Err: %7")
                                   .arg(FullBaseData.CurrSensorData.SensorData[6])
                                   .arg(FullBaseData.CurrSensorData.SensorData[7])
                                   .arg(FullBaseData.CurrSensorData.SensorData[8])
                                   .arg(FullBaseData.CurrSensorData.SensorData[9])
                                   .arg(FullBaseData.CurrSensorData.SensorData[10])
                                   .arg(FullBaseData.CurrSensorData.SensorData[11])
                                   .arg(FullBaseData.CurrSensorData.PosError);

    ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,1,new QTableWidgetItem(QString::number(FullBaseData.ucTimeStamp) ));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,3,new QTableWidgetItem(QString::number(FullBaseData.SyncId)));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,
                              new QTableWidgetItem((QString)BaseSensorDataP1.data() ) );

    ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,1,new QTableWidgetItem(QString::number(FullBaseData.ucTimeStamp) ));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,3,new QTableWidgetItem(QString::number(FullBaseData.SyncId)));
    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,
                              new QTableWidgetItem((QString)BaseSensorDataP2.data() ) );


    FrameCounter++;
}
/*********************************************************************************************************/
void MainWindow::BLE_LfAppBaseData(const QByteArray &value,BLE_MessageID_t BLE_MessID)
{
    static uint32_t PrevSyncId = 0U;
    static uint32_t FullFrameCounter = 0;
    uint8_t _inputSyncId = ((uint8_t)value.at(1)) ;
    static volatile uint8_t ExpectingFrameNumber= 0;


    static MainWindow::BLE_LfDataReport_t IncomingLfBaseData;


    if( (0 == ExpectingFrameNumber) && (BLE_MessID == BLE_MessageID_t::BLE_BaseDataReport_part1)  && (PrevSyncId != _inputSyncId))
    {
        IncomingLfBaseData.SyncId = _inputSyncId;
        IncomingLfBaseData.ucTimeStamp = ConvToUint32(value,2);
        IncomingLfBaseData.CurrMapData.WhLftSp = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6)) ;
        IncomingLfBaseData.CurrMapData.WhRhtSp =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10)) ;
        IncomingLfBaseData.CurrMapData.YawRate =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,14)) ;
        ExpectingFrameNumber = 1;
    }
    else if( (1 == ExpectingFrameNumber) && (BLE_MessID == BLE_MessageID_t::BLE_BaseDataReport_part2)  && (PrevSyncId == _inputSyncId))
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
        if( (2 == ExpectingFrameNumber)  && (BLE_MessID == BLE_MessageID_t::BLE_BaseDataReport_part3) && (PrevSyncId == _inputSyncId) )
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

                timer.start();
                RefreshErrorIndicatorView();
                qDebug() << "RefreshErrorIndicatorView TOOK: " << timer.elapsed() << "milliseconds";

                timer.start();
                UpdateDebugTableWithNewLfAppBaseData();
                qDebug() << "UpdateDebugTableWithNewLfAppBaseData TOOK: " << timer.elapsed() << "milliseconds";

                timer.start();
                MapPlot->LfGraph_AppendData(FullBaseData.CurrMapData.PosX,FullBaseData.CurrMapData.PosY);
                qDebug() << "MapGraph_AppendData TOOK: " << timer.elapsed() << "milliseconds";

                timer.start();

                YawRatePlot->LfGraph_AppendData((float)FullFrameCounter,FullBaseData.CurrMapData.YawRate);
                qDebug() << "YawRateGraph_AppendData TOOK: " << timer.elapsed() << "milliseconds";

//                SpdPlot->LfGraph_AppendData(FullFrameCounter,FullBaseData.CurrMapData.WhLftSp);
                SpdPlot->LfGraph_AppendData((float)FullFrameCounter,FullBaseData.CurrMapData.YawRate);
        }
        else
        {
                if(PrevSyncId != _inputSyncId)
                {
                    /*Ignore synchronization error detected*/
                    QString ExpectingFrameNumber_s = QString::number(ExpectingFrameNumber);
                    QString SyncErroMessage = QString("MapDatSyncErr cID:'%1'|pID:'%2'|mID:'%3' eeData:'%4' ")
                                                  .arg(_inputSyncId).arg(PrevSyncId).arg(BLE_MessID),arg(ExpectingFrameNumber_s);
                    ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
                    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,new QTableWidgetItem(SyncErroMessage));
                    ui->tableWidge_2->item(ui->tableWidge_2->rowCount() -1 , 4) -> setData(Qt::BackgroundRole, QColor (250,0,0));
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
/*********************************************************************************************************/
void MainWindow::BLE_DataHandler(const QByteArray &value){

    static volatile BLE_MessageID_t BLE_MessageID;
    BLE_MessageID = ((BLE_MessageID_t)value.at(0) );

    qDebug() <<  "NewFrameID:" << BLE_MessageID << "SyncID:" << ((uint8_t)value.at(1) );

    switch(BLE_MessageID)
    {
        case BLE_MessageID_t::BLE_CommunicationStatistics:
        {
            BLE_CommunicationStatistics_Handler(value);
            break;
        }

        case BLE_MessageID_t::BLE_BaseDataReport_part1:
        {
            BLE_LfAppBaseData(value,BLE_MessageID);
            break;
        }
        case BLE_MessageID_t::BLE_BaseDataReport_part2:
        {
            BLE_LfAppBaseData(value,BLE_MessageID);
            break;
        }
        case BLE_MessageID_t::BLE_BaseDataReport_part3:
        {
            BLE_LfAppBaseData(value,BLE_MessageID);
            break;
        }

        default:
        {
        break;
        }
    }


    static uint32_t ShifterDelay = 0;
    QElapsedTimer timer;
    timer.start();

    if( (ShifterDelay % 3) == 0)
        {
            ui->tableWidge_2->scrollToBottom();
        }
    qDebug() << " ui->tableWidge_2->scrollToBottom() TOOK: " << timer.elapsed() << "milliseconds";
}
/*********************************************************************************************************/
void MainWindow::on_BLE_SuspendFakeProdButton_clicked()
{
    //    BLE_StartFakeProducer
    char command[1];
    command[0] = (char)BLE_SuspendFakeProducer;
    QByteArray Helper = QByteArray::fromRawData(command,1);
    Helper.append("\n\r");
    bleConnection.writeData(Helper);
}
/*********************************************************************************************************/
void MainWindow::on_BLE_ActivFakeProdButton_clicked()
{
//    BLE_StartFakeProducer
    char command[1];
    command[0] = (char)BLE_StartFakeProducer;
    QByteArray Helper = QByteArray::fromRawData(command,1);
    Helper.append("\n\r");
    bleConnection.writeData(Helper);
}
/*********************************************************************************************************/
void MainWindow::BLE_connectDevice()
{
    //    ui->BLE_AutoConnCheckBox->isChecked();
    emit BLE_connectToDevice(ui->BLE_DetectedDeviceComboBox->currentIndex());
}
/*********************************************************************************************************/
void MainWindow::on_BLE_DisconnectButton_clicked()
{
    on_BLE_SuspendFakeProdButton_clicked();
    QThread::msleep(30);
    on_BLE_SuspendFakeProdButton_clicked();
    QThread::msleep(30);
    on_BLE_SuspendFakeProdButton_clicked();
    emit BLE_DisconnectDevice();
}
/*********************************************************************************************************/
void MainWindow::on_BLE_BlockSignalsCheckBox_stateChanged(int arg1)
{
    (void)arg1;
    if(ui->BLE_BlockSignalsCheckBox->isChecked()){
        emit BLE_BlockData(true);
    }
    else{
        emit BLE_BlockData(false);
    }
}
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/




