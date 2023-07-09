#include "mainwindow.h"
#include "ui_mainwindow.h"

/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QThread::currentThread()->setObjectName("Main Window Thread");


    ui->setupUi(this);

    /*All declared plots/ graph must be initialized!!!*/
    PlotMap.LfGraphInitialize(ui->MapViewWidget,QCPGraph::lsNone);
    PlotYawRate.LfGraphInitialize(ui->PlotYawRateW,QCPGraph::lsLine);
    PlotSpd.LfGraphInitialize(ui->PlotSpdW,QCPGraph::lsLine);
    PlotPosErr.LfGraphInitialize(ui->PlotPosErrW,QCPGraph::lsLine);
    PlotPidRegVal.LfGraphInitialize(ui->PlotPidRegValW,QCPGraph::lsLine);


    /*Initialize all needed connections for Bluetooth Data Manager*/
    BLE_InitializeQTConnections();

    /*Debug Table configure*/
    ui->DebugDataTable->setRowCount(1);
    ui->DebugDataTable->setColumnWidth(0,70);
    ui->DebugDataTable->setColumnWidth(1,70);
    ui->DebugDataTable->setColumnWidth(2,60);
    ui->DebugDataTable->setColumnWidth(3,60);
    ui->DebugDataTable->setColumnWidth(4,400);
    ui->DebugDataTable->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);


    /*Initialize dark theme*/
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
    PlotMap.LfGraph_ClearData();
    PlotSpd.LfGraph_ClearData();
    PlotYawRate.LfGraph_ClearData();
    PlotPosErr.LfGraph_ClearData();
    PlotPidRegVal.LfGraph_ClearData();
}

/*********************************************************************************************************/
void MainWindow::BLE_InitializeQTConnections(void)
{
    /*BLE Signals*/

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
        SIGNAL(BleDatMngrSignal_PlotMapUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotMapReplot() ));

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotYawRateUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotYawRateReplot() ));

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotSpdUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotSpdReplot() ));

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotPosErrUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotPosErrReplot() ));

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotPidRegValUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotPidRegValReplot() ));



    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotMapAppendData(float,float) )
        ,this
        ,SLOT(MainWinPlot_PlotMapAppendData(float,float) ));

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotYawRateAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotYawRateAppendData(uint32_t,float) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotSpdAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotSpdAppendData(uint32_t,float) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotPosErrAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotPosErrAppendData(uint32_t,float) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_PlotPidRegValAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotPidRegValAppendData(uint32_t,float) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_DebugTable_InsertDataRow(uint32_t,uint32_t,uint8_t,QString,QColor) )
        ,this
        ,SLOT(MainWin_DebugTable_InsertDataRow(uint32_t,uint32_t,uint8_t,QString,QColor) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_DebugTable_ScrollToBottom() )
        ,this
        ,SLOT(MainWin_DebugTable_ScrollToBottom() ) );



    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_RefreshErrorIndicatorView(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
                                                          uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
                                                          uint8_t,uint8_t,
                                                          float) )
        ,this
        ,SLOT(MainWin_RefreshErrorIndicatorView(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
                                               uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
                                               uint8_t,uint8_t,
                                               float) )
                                                        );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_CommunicationStatisticsUpdate(uint32_t,uint16_t,uint16_t,uint16_t,uint16_t) )
        ,this
        ,SLOT(MainWin_CommunicationStatisticsUpdate(uint32_t,uint16_t,uint16_t,uint16_t,uint16_t) ) );

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
void MainWindow::MainWin_RefreshErrorIndicatorView( uint8_t S0,uint8_t S1,uint8_t S2,uint8_t S3,uint8_t S4,uint8_t S5,
                                            uint8_t S6,uint8_t S7,uint8_t S8,uint8_t S9,uint8_t S10,uint8_t S11,
                                            uint8_t RightLinePosConfif,uint8_t LeftLinePosConfif,
                                            float PosError)
{

    ui->S1_label->setText(QString::number(S0));
    ui->S2_label->setText(QString::number(S1));
    ui->S3_label->setText(QString::number(S2));
    ui->S4_label->setText(QString::number(S3));
    ui->S5_label->setText(QString::number(S4));
    ui->S6_label->setText(QString::number(S5));
    ui->S7_label->setText(QString::number(S6));
    ui->S8_label->setText(QString::number(S7));
    ui->S9_label->setText(QString::number(S8));
    ui->S10_label->setText(QString::number(S9));
    ui->S11_label->setText(QString::number(S10));
    ui->S12_label->setText(QString::number(S11));

    ui->ConfL_Right->setText(QString::number(RightLinePosConfif) );
    ui->ConfL_Left->setText(QString::number(LeftLinePosConfif) );
    ui->PositionErrorLabel->setText(QString::number(PosError) );



    QVariant variant= QColor (255,0,0);
    QString colcode = variant.toString();
    ui->PositionErrorLabel->setAutoFillBackground(true);
    ui->PositionErrorLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

    QFont f( "Arial", 18, QFont::Bold);
    ui->PositionErrorLabel->setFont(f);



    if(S0 > 60)
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

    if(S1 > 60)
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

    if(S2 > 60)
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

    if(S3 > 60)
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

    if(S4 > 60)
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


    if(S5 > 60)
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


    if(S6 > 60)
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

    if(S7 > 60)
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


    if(S8 > 60)
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


    if(S9 > 60)
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

    if(S10 > 60)
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

    if(S11 > 60)
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
void MainWindow::MainWin_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor )
{
    QColor QColorHelper = QColor(255,255,255); /*QColor default arg. value*/


        ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );

        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(ucTimeStamp) ));
        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem(QString::number(SyncId)));
        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
                                  new QTableWidgetItem((QString)DecodedDataString.data() ) );


        QDateTime t = QDateTime::currentDateTime();
        QString st = t.toString("yy/MM/dd HH:mm:ss.zzz");

        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,0,new QTableWidgetItem(st)  ) ;

        if(QColorHelper != RowColor) /*default value*/
        {
            ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 1) -> setData(Qt::BackgroundRole, RowColor);
            ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 2) -> setData(Qt::BackgroundRole, RowColor);
            ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 3) -> setData(Qt::BackgroundRole, RowColor);
            ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 4) -> setData(Qt::BackgroundRole, RowColor);
        }
}

void MainWindow::MainWin_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
        uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter)
{
        (void)RetransmissionCounter;

        QString StatData = QString("ucT:%1ms |BuffRemSize:%2 |OverFlowC:%3 |FrmCnt: %4")
                               .arg(ucTimeStamp).arg(RingBufferRemainingSize)
                               .arg(RingBufferOverFlowCounter).arg(TransmisstedMessagesCounter);

        ui->Label_CommunicationStat->setText(StatData);
}


void MainWindow::MainWin_DebugTable_ScrollToBottom()
{
    ui->DebugDataTable->scrollToBottom();
    BleInputDataProcessingWrapper.DebugTableScrollingBottonMutex.lock();
    BleInputDataProcessingWrapper.DebugTableScrollingBottomIsActivState = false;
    BleInputDataProcessingWrapper.DebugTableScrollingBottonMutex.unlock();
}


/*********************************************************************************************************/

void MainWindow::MainWinPlot_PlotPosErrReplot(void)
{
    PlotPosErr.LfGraph_UpdateReplot();

    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.PosErrPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotPidRegValReplot()
{
    PlotPidRegVal.LfGraph_UpdateReplot();

    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.PidRegValPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotPosErrAppendData(uint32_t FrameId, float PossErrValue)
{
    PlotPosErr.LfGraph_AppendData(FrameId,PossErrValue);
}

void MainWindow::MainWinPlot_PlotPidRegValAppendData(uint32_t FrameId, float PidRegVal)
{
    PlotPidRegVal.LfGraph_AppendData(FrameId,PidRegVal);
}


void MainWindow::MainWinPlot_PlotMapAppendData(float PosX, float PosY)
{
    PlotMap.LfGraph_AppendData(PosX,PosY);

}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotYawRateAppendData(uint32_t FrameId, float YrValue)
{
    PlotYawRate.LfGraph_AppendData((float)FrameId,YrValue);
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotSpdAppendData(uint32_t FrameId, float SpdValue)
{
    PlotSpd.LfGraph_AppendData((float)FrameId,SpdValue);
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotMapReplot(void)
{

//    QElapsedTimer timer;
//    timer.start();
    PlotMap.LfGraph_UpdateReplot();
//    qDebug() << "MainWinPlot_PlotMapReplot TOOK: " << timer.elapsed() << "milliseconds";
    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.MapPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotYawRateReplot(void)
{
//    qInfo() << this << "MainWinPlot_PlotYawRateReplot Thread:  " << QThread::currentThread();
    PlotYawRate.LfGraph_UpdateReplot();

    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.YawRatePlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotSpdReplot(void)
{
//    qInfo() << this << "MainWinPlot_PlotSpdReplot Thread:  " << QThread::currentThread();
    PlotSpd.LfGraph_UpdateReplot();

    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.SpdPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
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


void MainWindow::on_EnableBaseDataLogging_clicked(bool checked)
{
    if(true == checked)
    {
        BleInputDataProcessingWrapper.DebugTable_BaseDataLoggingState = true;
    }
}


void MainWindow::on_DebugTable_DisableBaseDataLogging_clicked(bool checked)
{
    if(true == checked)
    {
        BleInputDataProcessingWrapper.DebugTable_BaseDataLoggingState = false;
    }
}



