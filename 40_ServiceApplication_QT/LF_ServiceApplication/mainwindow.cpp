#include "mainwindow.h"
#include "ui_mainwindow.h"

/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QThread::currentThread()->setObjectName("Main Window Thread");


    ui->setupUi(this);

    /*All declared graph must be initialized!!!*/
    BleInputDataProcessingWrapper.MapPlot.LfGraphInitialize(ui->MapViewWidget,QCPGraph::lsNone);
    BleInputDataProcessingWrapper.YawRatePlot.LfGraphInitialize(ui->YawRatePlotW,QCPGraph::lsLine);
    BleInputDataProcessingWrapper.SpdPlot.LfGraphInitialize(ui->SpdPlotW,QCPGraph::lsLine);


    /**/
    BLE_InitializeQTConnections();




    /*Debug Table configure*/
    ui->DebugDataTable->setRowCount(1);
    ui->DebugDataTable->setColumnWidth(0,50);
    ui->DebugDataTable->setColumnWidth(1,50);
    ui->DebugDataTable->setColumnWidth(2,28);
    ui->DebugDataTable->setColumnWidth(3,10);
    ui->DebugDataTable->setColumnWidth(4,400);
    ui->DebugDataTable->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);





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

    delete ui;

    qDebug("Reached end");
}




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
    connect(this, SIGNAL(BLE_BlockData(bool)),&BleInputDataProcessingWrapper.bleConnection,SLOT(BlockData(bool)));
    /* Signal and Slots */
    /* Search Button */
    connect(ui->BLE_ScanButton, SIGNAL(clicked()),&BleInputDataProcessingWrapper.bleConnection, SLOT(startScan() ));

    /* Connect Button */
    connect(ui->BLE_ConnectButton,SIGNAL(clicked()), this, SLOT(BLE_connectDevice()));
    connect(this, SIGNAL(BLE_DisconnectDevice() ),&BleInputDataProcessingWrapper.bleConnection,SLOT(DisconnectDevice() ));
    //    /* Bleutooth States */

    connect(
            &BleInputDataProcessingWrapper.bleConnection,
            SIGNAL(changedState(bluetoothleUART::bluetoothleState))
            ,this
            ,SLOT(BLE_changedState(bluetoothleUART::bluetoothleState)));


    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_MapPlotUpdate() )
        ,this
        ,SLOT(MainWinPlot_MapUpdate() ));


    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_YawRatePlotUpdate() )
        ,this
        ,SLOT(MainWinPlot_YawRateUpdate() ));

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_SpdPlotUpdate() )
        ,this
        ,SLOT(MainWinPlot_SpdUpdate() ));



//    connect(
//        &BleInputDataProcessingWrapper,
//        SIGNAL(LfAppBaseDataSignal_WrS(QByteArray,BleDataManager::BLE_MessageID_t) ),
//        this,
//        SLOT(BLE_LfAppBaseData(QByteArray,BleDataManager::BLE_MessageID_t) ) );


//    connect(
//        &BleInputDataProcessingWrapper,
//        SIGNAL(LfAppCommunicationStatisticsSignal_WrS(QByteArray) ),
//        this,
//        SLOT(BLE_UpdateDebugTableWithNewLfAppBaseData()   ) );
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
        BleInputDataProcessingWrapper.bleConnection.getDeviceList(FoundDevices);

        if(!FoundDevices.empty()){

            ui->BLE_DetectedDeviceComboBox->addItem(FoundDevices.at(deviceCounter_l));
            /* Initialise Slot startConnect(int) -> button press ui->BLE_ConnectButton */
            connect(this, SIGNAL(BLE_connectToDevice(int)),&BleInputDataProcessingWrapper.bleConnection,SLOT(startConnect(int)));

            ui->BLE_ConnectButton->setEnabled(true);
            ui->BLE_ScanButton->setEnabled(true);
            ui->BLE_DetectedDeviceComboBox->setEnabled(true);

            QString AutoConnDevName = ui->BLE_AutoConnDevNameL->text();

            if( ui->BLE_DetectedDeviceComboBox->itemText(deviceCounter_l) ==  AutoConnDevName
                && ui->BLE_AutoConnCheckBox->isChecked() )
            {
                ui->BLE_DetectedDeviceComboBox->setCurrentIndex(deviceCounter_l);
                ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,new QTableWidgetItem(QString("Expected dev. name Matched: Autoconnection ") ));
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
/*********************************************************************************************************/
void MainWindow::RefreshErrorIndicatorView(void)
{
//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[0] > 50)

//    ui->S1_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[0]));
//    ui->S2_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[1]));
//    ui->S3_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[2]));
//    ui->S4_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[3]));
//    ui->S5_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[4]));
//    ui->S6_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[5]));
//    ui->S7_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[6]));
//    ui->S8_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[7]));
//    ui->S9_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[8]));
//    ui->S10_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[9]));
//    ui->S11_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[10]));
//    ui->S12_label->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.SensorData[11]));

//    ui->PositionErrorLabel->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.LastRightLinePosConfidence) );
//    ui->PositionErrorLabel->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.LastLeftLinePosConfidence) );
//    ui->PositionErrorLabel->setText(QString::number(BleDataManager::FullBaseData.CurrSensorData.PosError) );



//    QVariant variant= QColor (255,0,0);
//    QString colcode = variant.toString();
//    ui->PositionErrorLabel->setAutoFillBackground(true);
//    ui->PositionErrorLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//    QFont f( "Arial", 18, QFont::Bold);
//    ui->PositionErrorLabel->setFont(f);



//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[0] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S1_label->setAutoFillBackground(true);
//        ui->S1_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S1_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S1_label->setAutoFillBackground(true);
//        ui->S1_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S1_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[1] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S2_label->setAutoFillBackground(true);
//        ui->S2_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S2_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S2_label->setAutoFillBackground(true);
//        ui->S2_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S2_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[2] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S3_label->setAutoFillBackground(true);
//        ui->S3_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S3_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S3_label->setAutoFillBackground(true);
//        ui->S3_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S3_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[3] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S4_label->setAutoFillBackground(true);
//        ui->S4_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S4_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S4_label->setAutoFillBackground(true);
//        ui->S4_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S4_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[4] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S5_label->setAutoFillBackground(true);
//        ui->S5_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S5_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S5_label->setAutoFillBackground(true);
//        ui->S5_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S5_label->setFont(f);
//    }


//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[5] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S6_label->setAutoFillBackground(true);
//        ui->S6_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S6_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S6_label->setAutoFillBackground(true);
//        ui->S6_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S6_label->setFont(f);
//    }


//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[6] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S7_label->setAutoFillBackground(true);
//        ui->S7_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S7_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S7_label->setAutoFillBackground(true);
//        ui->S7_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S7_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[7] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S8_label->setAutoFillBackground(true);
//        ui->S8_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S8_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S8_label->setAutoFillBackground(true);
//        ui->S8_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S8_label->setFont(f);
//    }


//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[8] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S9_label->setAutoFillBackground(true);
//        ui->S9_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S9_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S9_label->setAutoFillBackground(true);
//        ui->S9_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S9_label->setFont(f);
//    }


//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[9] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S10_label->setAutoFillBackground(true);
//        ui->S10_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S10_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S10_label->setAutoFillBackground(true);
//        ui->S10_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S10_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[10] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S11_label->setAutoFillBackground(true);
//        ui->S11_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S11_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S11_label->setAutoFillBackground(true);
//        ui->S11_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S11_label->setFont(f);
//    }

//    if(BleDataManager::FullBaseData.CurrSensorData.SensorData[11] > 60)
//    {
//        QVariant variant= QColor (255,120,120);
//        QString colcode = variant.toString();
//        ui->S12_label->setAutoFillBackground(true);
//        ui->S12_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S12_label->setFont(f);
//    }
//    else
//    {
//        QVariant variant= QColor (255,255,255);
//        QString colcode = variant.toString();
//        ui->S12_label->setAutoFillBackground(true);
//        ui->S12_label->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

//        QFont f( "Arial", 13, QFont::Bold);
//        ui->S12_label->setFont(f);
//    }

}
/*********************************************************************************************************/
void MainWindow::BLE_UpdateDebugTableWithNewLfAppBaseData(void)
{
//    static uint16_t FrameCounter = 0U;

//    QString RgtWhlSpdSimu_s = QString::number(BleDataManager::FullBaseData.CurrMapData.WhLftSp,'f',3);
//    QString LftWhlSpdSimu_s = QString::number(BleDataManager::FullBaseData.CurrMapData.WhRhtSp,'f',3);
//    QString BaseMapData = QString("pX: %1 |pY: %2 |L_sp: %3 |R_Sp: %4| Yr: %5")
//                              .arg(BleDataManager::FullBaseData.CurrMapData.PosX).arg(BleDataManager::FullBaseData.CurrMapData.PosY)
//                              .arg(LftWhlSpdSimu_s).arg(RgtWhlSpdSimu_s).arg(BleDataManager::FullBaseData.CurrMapData.YawRate) ;

//    ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(BleDataManager::FullBaseData.ucTimeStamp) ));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem(QString::number(BleDataManager::FullBaseData.SyncId)));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
//                              new QTableWidgetItem((QString)BaseMapData.data() ) );


//    QString BaseSensorDataP1 = QString("S0: %1 |S1: %2 |S2: %3 |S3: %4 |S4: %5 |S5: %6 |Err: %7")
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[0])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[1])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[2])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[3])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[4])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[5])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.PosError);

//    QString BaseSensorDataP2 = QString("S6: %1 |S7: %2 |S8: %3 |S9: %4 |S10: %5 |S11: %6 |Err: %7")
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[6])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[7])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[8])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[9])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[10])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.SensorData[11])
//                                   .arg(BleDataManager::FullBaseData.CurrSensorData.PosError);

//    ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(BleDataManager::FullBaseData.ucTimeStamp) ));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem(QString::number(BleDataManager::FullBaseData.SyncId)));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
//                              new QTableWidgetItem((QString)BaseSensorDataP1.data() ) );

//    ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(BleDataManager::FullBaseData.ucTimeStamp) ));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem(QString::number(BleDataManager::FullBaseData.SyncId)));
//    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
//                              new QTableWidgetItem((QString)BaseSensorDataP2.data() ) );


//    FrameCounter++;
}
/*********************************************************************************************************/


void MainWindow::MainWinPlot_MapUpdate(void)
{
//    qInfo() << this << "MainWinPlot_MapUpdate Thread:  " << QThread::currentThread();
    BleInputDataProcessingWrapper.MapPlot.LfGraph_UpdateReplot();

}

void MainWindow::MainWinPlot_YawRateUpdate(void)
{
//    qInfo() << this << "MainWinPlot_YawRateUpdate Thread:  " << QThread::currentThread();
    BleInputDataProcessingWrapper.YawRatePlot.LfGraph_UpdateReplot();
}

void MainWindow::MainWinPlot_SpdUpdate(void)
{
//    qInfo() << this << "MainWinPlot_SpdUpdate Thread:  " << QThread::currentThread();
    BleInputDataProcessingWrapper.SpdPlot.LfGraph_UpdateReplot();
}


/*********************************************************************************************************/
void MainWindow::on_BLE_SuspendFakeProdButton_clicked()
{
    //    BLE_StartFakeProducer
    char command[1];
    command[0] = (char)BleDataManager::BLE_SuspendFakeProducer;
    QByteArray Helper = QByteArray::fromRawData(command,1);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}
/*********************************************************************************************************/
void MainWindow::on_BLE_ActivFakeProdButton_clicked()
{
//    BLE_StartFakeProducer
    char command[1];
    command[0] = (char)BleDataManager::BLE_StartFakeProducer;
    QByteArray Helper = QByteArray::fromRawData(command,1);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
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




