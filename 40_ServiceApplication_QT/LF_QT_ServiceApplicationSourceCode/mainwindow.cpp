#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QUrl>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>

/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QThread::currentThread()->setObjectName("Main Window Thread");

    ui->setupUi(this);

    this->setWindowTitle("Line Follower Ligth QT Service Application");


    /*All declared plots/ graph must be initialized!!!*/
    PlotMap.LfGraphInitialize(ui->MapViewWidget,QCPGraph::lsNone);
    PlotYawRate.LfGraphInitialize(ui->PlotYawRateW,QCPGraph::lsLine);
    PlotSpd.LfGraphInitialize(ui->PlotSpdW,QCPGraph::lsLine);
    PlotPosErr.LfGraphInitialize(ui->PlotPosErrW,QCPGraph::lsLine);
    PlotPidRegVal.LfGraphInitialize(ui->PlotPidRegValW,QCPGraph::lsLine);
    PlotTrvDistance.LfGraphInitialize(ui->PlotTrvDistanceW,QCPGraph::lsLine);
    PlotOrientation.LfGraphInitialize(ui->PlotOrientationW,QCPGraph::lsLine);
    PlotLinePosConfidence.LfGraphInitialize(ui->PlotLinePosConfW,QCPGraph::lsLine);

    connect(&PlotMap, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotYawRate, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotSpd, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotPosErr, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotPidRegVal, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotTrvDistance, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotOrientation, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
    connect(&PlotLinePosConfidence, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));


    /*Initialize all needed connections for Bluetooth Data Manager*/
    BLU_InitializeQTConnections();
    ConfigureTextLineAndNvMConnections();


    addJoyStick(ui->RobotMoverXYGridLayout);


    /*Debug Table configure*/
    ui->DebugDataTable->setRowCount(0);
    ui->DebugDataTable->setColumnWidth(0,10);
    ui->DebugDataTable->setColumnWidth(1,10);
    ui->DebugDataTable->setColumnWidth(2,10);
    ui->DebugDataTable->setColumnWidth(3,10);
    ui->DebugDataTable->setColumnWidth(4,600);
    ui->DebugDataTable->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);


    QPalette pal = ui->BLU_RobotStart_Button->palette();
    pal.setColor(QPalette::Button, QColor(Qt::blue));
    ui->BLU_RobotStart_Button->setAutoFillBackground(true);
    ui->BLU_RobotStart_Button->setPalette(pal);
    ui->BLU_RobotStart_Button->update();


    MainWin_DrawOrientationIndicator( 3.14F / 2.0F);

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


    QSettings settings("LfServiceApp", "BluDeviceName");
//    settings.setValue("CurrDeviceName", "Franek");
    QVariant CurrDevName = settings.value("CurrDeviceName");
    QString NewLineEditText = CurrDevName.toString();
//    qDebug() << NewLineEditText ;

    ui->BLU_AutoConnDevNameL->setText(NewLineEditText);


//    if(qApp->arguments().count() > 0)
//    {
//        qDebug() << "argumentsCount" << qApp->arguments().at(0);
//    }

    if(qApp->arguments().count() > 1  && qApp->arguments().at(1).endsWith(".lfp") ==true)
    {
        CurrentLfProjectFilePath = qApp->arguments().at(1);
        QString LoadedProjectInfo = QString("DataLoadedFromProjectFile: %1").arg(CurrentLfProjectFilePath);
        MainWin_DebugTable_InsertDataRow(0, 0, 0, LoadedProjectInfo);

        LoadDataLineFollowerProjecrOrJson(CurrentLfProjectFilePath);
    }
}


/*********************************************************************************************************/
MainWindow::~MainWindow()
{
    on_BLU_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    on_BLU_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    on_BLU_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    /*Send the command 3x to be sure that fakeProducer will be stopped, if not then re-connection may be impossible
     * - HW reset may be required
    */

//    TODO: Veriyfy? emit BLU_DisconnectDevice();

    QSettings settings("LfServiceApp", "BluDeviceName");
    QString NewSearchedBluDeviceName = ui->BLU_AutoConnDevNameL->text();
    settings.setValue("CurrDeviceName", NewSearchedBluDeviceName);

    delete ui;

    qDebug("Reached end");
}

void MainWindow::changeEvent( QEvent* e )
{
    if( e->type() == QEvent::WindowStateChange )
    {
        qDebug() << "this->windowState()" << this->windowState();

        QWindowStateChangeEvent* event = static_cast< QWindowStateChangeEvent* >( e );

        if( event->oldState() & Qt::WindowMinimized )
        {
            qDebug() << "Window restored (to normal or maximized state)!";
        }
        else if( event->oldState() == Qt::WindowNoState && this->windowState() == Qt::WindowMaximized )
        {
            qDebug() << "Window Maximized!";

        }
        else if(this->windowState() == Qt::WindowNoState && event->oldState() == Qt::WindowMaximized){
            qDebug() << "Window restored to normal from maximazed state";
        }
    }
}

void MainWindow::addJoyStick(QLayout *layout_, JoyType type)
{
    QQuickView *view = new QQuickView();
    /* NB: We load the QML from a .qrc file becuase the Qt build step
     * that packages the final .app on Mac forgets to add the QML
     * if you reference it directly
     */
    view->setSource(QUrl("qrc:/res/virtual_joystick.qml"));

    /* Enable transparent background on the QQuickView
     * Note that this currently does not work on Windows
     */
#ifndef _WIN32
    view->setClearBeforeRendering(true);
    view->setColor(QColor(Qt::transparent));
#endif

    // Attach to the 'mouse moved' signal
    auto *root = view->rootObject();
    if (type == HorizontalOnly)
        root->setProperty("horizontalOnly", true);
    else if (type == VerticalOnly)
        root->setProperty("verticalOnly", true);
    connect(
        root,
        SIGNAL(joystick_moved(double, double)),
        this,
        SLOT(joystick_moved(double, double))
        );

    // Create a container widget for the QQuickView
    QWidget *container = QWidget::createWindowContainer(view, this);
    container->setMinimumSize(160, 160);
    container->setMaximumSize(160, 160);
    container->setFocusPolicy(Qt::TabFocus);
    layout_->addWidget(container);
}

/**
 * @brief MainWindow::mouse_moved Called when the virtual joystick is moved
 * @param x Mouse x position
 * @param y Mouse y position
 */
void MainWindow::joystick_moved(double x, double y) {

    float VecValX= (float)x;
    float VecValY= (float)y;
    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2];

    command[0] = (char)BluDataManager::BLU_NvM_ManualCntrlCommand;
    std::memcpy(&command[2],  &VecValX, sizeof(float));
    std::memcpy(&command[6],  &VecValY, sizeof(float));

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

//Debug/ test
//    float ExpectedOrientation = ( (float)atan2(VecValY,VecValX) );
//    if(ExpectedOrientation < (0.0F) ){
//        ExpectedOrientation =  (3.14) + (3.14 - fabs(ExpectedOrientation));
//        /*To get full circle*/
//    }
//    MainWin_DrawOrientationIndicator(ExpectedOrientation);

    //qDebug() << "VecValX:" << VecValX << "VecValY: " << VecValY;
}

void MainWindow::ConfigureTextLineAndNvMConnections(void)
{
    dblValidator.setNotation(QDoubleValidator::StandardNotation);
    dblValidator.setLocale(QLocale::C);

    ui->ErrW1_Text->setValidator(&dblValidator);
    ui->ErrW2_Text->setValidator(&dblValidator);
    ui->ErrW3_Text->setValidator(&dblValidator);
    ui->ErrW4_Text->setValidator(&dblValidator);
    ui->ErrW5_Text->setValidator(&dblValidator);
    ui->ErrW6_Text->setValidator(&dblValidator);
    ui->ErrW7_Text->setValidator(&dblValidator);
    ui->ErrW8_Text->setValidator(&dblValidator);
    ui->ErrW9_Text->setValidator(&dblValidator);
    ui->ErrW10_Text->setValidator(&dblValidator);
    ui->ErrW11_Text->setValidator(&dblValidator);
    ui->ErrWM_Text->setValidator(&dblValidator);

    ui->ExpectedAvSpdText->setValidator(&dblValidator);
    ui->PID_KP_text->setValidator(&dblValidator);
    ui->PID_KI_text->setValidator(&dblValidator);
    ui->PID_KD_text->setValidator(&dblValidator);
    ui->ProbeTimeText->setValidator(&dblValidator);

    ui->TextWheelBase->setValidator(&dblValidator);
    ui->TextOneImpDist->setValidator(&dblValidator);

    ui->textRightAgKp->setValidator(&dblValidator);
    ui->textRightAgKd->setValidator(&dblValidator);
    ui->textRightAgBaseSpd->setValidator(&dblValidator);
    ui->textRightAgMaxYr->setValidator(&dblValidator);
    ui->textRightBrThr->setValidator(&dblValidator);
    ui->textRightBrTim->setValidator(&dblValidator);
    ui->textRightOriCh->setValidator(&dblValidator);
    ui->textRightOriChAftBr->setValidator(&dblValidator);
    ui->textRightAgPrTim->setValidator(&dblValidator);

    ui->TextPwmToSpAFacL->setValidator(&dblValidator);
    ui->TextPwmToSpAFacR->setValidator(&dblValidator);
    ui->TextPwmToSpBFacL->setValidator(&dblValidator);
    ui->TextPwmToSpBFacR->setValidator(&dblValidator);

    ui->SpdProfileTrvD1->setValidator(&dblValidator);
    ui->SpdProfileTrvD2->setValidator(&dblValidator);
    ui->SpdProfileTrvD3->setValidator(&dblValidator);
    ui->SpdProfileTrvD4->setValidator(&dblValidator);
    ui->SpdProfileTrvD5->setValidator(&dblValidator);
    ui->SpdProfileTrvD6->setValidator(&dblValidator);
    ui->SpdProfileTrvD7->setValidator(&dblValidator);
    ui->SpdProfileTrvD8->setValidator(&dblValidator);
    ui->SpdProfileTrvD9->setValidator(&dblValidator);
    ui->SpdProfileTrvD10->setValidator(&dblValidator);
    ui->SpdProfileTrvD11->setValidator(&dblValidator);

    ui->SpdProfileBspd1->setValidator(&dblValidator);
    ui->SpdProfileBspd2->setValidator(&dblValidator);
    ui->SpdProfileBspd3->setValidator(&dblValidator);
    ui->SpdProfileBspd4->setValidator(&dblValidator);
    ui->SpdProfileBspd5->setValidator(&dblValidator);
    ui->SpdProfileBspd6->setValidator(&dblValidator);
    ui->SpdProfileBspd7->setValidator(&dblValidator);
    ui->SpdProfileBspd8->setValidator(&dblValidator);
    ui->SpdProfileBspd9->setValidator(&dblValidator);
    ui->SpdProfileBspd10->setValidator(&dblValidator);
    ui->SpdProfileBspd11->setValidator(&dblValidator);

    connect(&NvM_ErrWeigthUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_ErrWeigthUpdateDelayTimerTimeout()));
    connect(&NvM_PidDatahUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_PidDatahUpdateDelayTimerTimeout()));
    connect(&NvM_rAgDatahUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_rAgDatahUpdateDelayTimerTimeout()));
    connect(&NvM_VehCfghUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_VehCfgDataUpdateDelayTimerTimeout()));
    connect(&NvM_MotorsFactorsUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_MotorsFactorsDataUpdateDelayTimerTimeout()));
    connect(&NvM_EncoderCfgUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_EncodersConfigDataUpdateDelayTimerTimeout()));
    connect(&NvM_SpeedProfileUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_ProfileSpeedConfigDataUpdateDelayTimerTimeout()));

}



/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
void MainWindow::MainWin_bluetoothSlotDeviceDiscovered(QString name)
{
//    qDebug() << "MainWin_bluetoothSlotDeviceDiscovered  called";

    ui->BLU_ConnectButton->setEnabled(true);
    ui->BLU_DetectedDeviceComboBox->setEnabled(true);

    ui->BLU_StatusLabel->setText("State:NewDevDet");

    ui->BLU_DetectedDeviceComboBox->addItem(name);

    QString AutoConnDevName = ui->BLU_AutoConnDevNameL->text();

    if( name == AutoConnDevName && ui->BLU_AutoConnCheckBox->isChecked() )
    {
        ui->BLU_DetectedDeviceComboBox->setCurrentIndex( ui->BLU_DetectedDeviceComboBox->count() - 1 );

        ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
                                    new QTableWidgetItem(QString("DevNameMatched:Connecting ") ));

        emit MainWin_bluetootSignalConnectToDeviceByName(AutoConnDevName);
    }
    else
    {
        ui->statusbar->showMessage("Please select BLE device",1000);
    }
}

void MainWindow::MainWin_bluetoothSlotDiscoveryFinished(void)
{
    ui->BLU_ConnectButton->setEnabled(true);
    ui->BLU_ScanButton->setEnabled(true);
    ui->BLU_DetectedDeviceComboBox->setEnabled(true);

    ui->statusbar->showMessage("DiscoveryFinished",1000);
//    ui->BLU_StatusLabel->setText("DiscoveryFinished");
}

void MainWindow::MainWin_bluetoothSlotConnectingStart()
{
    ui->BLU_StatusLabel->setText("State:Connecting");
    ui->statusbar->showMessage("Connecting to device...",1000);
}

void MainWindow::MainWin_bluetoothSlotConnectionEstablished(void)
{
    ui->statusbar->showMessage("Aquire data",1000);

    ui->BLU_StatusLabel->setText("State:ConnReadyAcqData");
    QVariant variant= QColor (220,255,220);
    QString colcode = variant.toString();
    ui->BLU_StatusLabel->setAutoFillBackground(true);
    ui->BLU_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

    if(false == NvM_DataLoadedFromExternalSourceFlag)
    {
        NvM_DataLoadedFromExternalSourceFlag = true;
        ReadNvMDataFromLineFollower();
    }

}

void MainWindow::MainWin_bluetoothSlotConnectionInterrupted(void)
{

    ui->statusbar->showMessage("Connection interrupted, why?",1000);


    ui->BLU_StatusLabel->setText("State:Disconnected");
    QVariant variant= QColor (255,255,255);
    QString colcode = variant.toString();
    ui->BLU_StatusLabel->setAutoFillBackground(true);
    ui->BLU_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");
}

void MainWindow::on_BLU_ScanButton_clicked()
{
    ui->BLU_StatusLabel->setText("State:Scanning");
    ui->BLU_DetectedDeviceComboBox->clear();
    ui->BLU_ConnectButton->setEnabled(false);
    ui->BLU_ScanButton->setEnabled(false);
    ui->BLU_DetectedDeviceComboBox->setEnabled(true);
    ui->statusbar->showMessage("Searching for bluetooth devices...",1000);
    emit MainWin_bluetoothSignalStartDiscoveryDevices();
}

void MainWindow::on_BLU_ConnectButton_clicked()
{
    QString NewAutoConnDevName = ui->BLU_DetectedDeviceComboBox->itemText(ui->BLU_DetectedDeviceComboBox->currentIndex());
    ui->BLU_AutoConnDevNameL->setText(NewAutoConnDevName);
    emit MainWin_bluetootSignalConnectToDeviceByName(NewAutoConnDevName);
}

void MainWindow::on_BLU_DisconnectButton_clicked()
{

    on_BLU_RobotStop_Button_clicked();
    QThread::msleep(30);
    on_BLU_RobotStop_Button_clicked();
    QThread::msleep(30);
    on_BLU_RobotStop_Button_clicked();
    emit MainWin_bluetoothDisconnect();

    ui->BLU_StatusLabel->setText("State:Disconnected");
    ui->BLU_ConnectButton->setEnabled(true);
    ui->BLU_ScanButton->setEnabled(true);
    QVariant variant= QColor (255,255,255);
    QString colcode = variant.toString();
    ui->BLU_StatusLabel->setAutoFillBackground(true);
    ui->BLU_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");
}


/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

void MainWindow::BLU_InitializeQTConnections(void)
{

    /*Bluetooth connect connections start*/
    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL(bluetoothSignalDeviceDiscovered(QString) )
        ,this
        ,SLOT(MainWin_bluetoothSlotDeviceDiscovered(QString) )   );

    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL(bluetoothSignalDiscoveryFinished() )
        ,this
        ,SLOT(MainWin_bluetoothSlotDiscoveryFinished() ));


    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL(bluetoothSignalConnectingStart() )
        ,this
        ,SLOT(MainWin_bluetoothSlotConnectingStart() ));


    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL( bluetoothSignalConnectionEstablished() )
        ,this
        ,SLOT(MainWin_bluetoothSlotConnectionEstablished() ));


    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL( bluetoothSignalConnectionInterrupted() )
        ,this
        ,SLOT(MainWin_bluetoothSlotConnectionInterrupted() ));
    /**/
    connect(
        this,
        SIGNAL( MainWin_bluetoothSignalStartDiscoveryDevices() )
        ,&BluInputDataProcessingWrapper.bleutoothClassicConnection
        ,SLOT(bluetoothStartDiscoveryDevices() ) );

    connect(
        this,
        SIGNAL( MainWin_bluetoothDisconnect() )
        ,&BluInputDataProcessingWrapper.bleutoothClassicConnection
        ,SLOT(bluetoothDisconnect() ) );

    connect(
        this,
        SIGNAL( MainWin_bluetootSignalConnectToDeviceByName(QString) )
        ,&BluInputDataProcessingWrapper.bleutoothClassicConnection
        ,SLOT(bluetootConnectToDeviceByName(QString) ) );
    /*Bluetooth connect connections end*/



    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotMapUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotMapReplot() ));

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotYawRateUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotYawRateReplot() ));

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotSpdUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotSpdReplot() ));


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotPosErrUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotPosErrReplot() ));

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotPidRegValUpdate() )
        ,this
        ,SLOT(MainWinPlot_PlotPidRegValReplot() ));



    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotMapAppendData(float,float) )
        ,this
        ,SLOT(MainWinPlot_PlotMapAppendData(float,float) ));

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotYawRateAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotYawRateAppendData(uint32_t,float) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotSpdAppendData(uint32_t,float,float) )
        ,this
        ,SLOT(MainWinPlot_PlotSpdAppendData(uint32_t,float,float) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotPosErrAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotPosErrAppendData(uint32_t,float) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotPidRegValAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotPidRegValAppendData(uint32_t,float) ) );





    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotOrientationAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotOrientationAppendData(uint32_t,float) ) );


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotOrientationReplot() )
        ,this
        ,SLOT(MainWinPlot_PlotOrientationReplot() ));

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotTrvDistanceAppendData(uint32_t,float) )
        ,this
        ,SLOT(MainWinPlot_PlotTrvDistanceAppendData(uint32_t,float) ) );


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotTrvDistanceReplot() )
        ,this
        ,SLOT(MainWinPlot_PlotTrvDistanceReplot() ));


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotPosConfidenceAppendData(uint32_t, uint8_t, uint8_t) )
        ,this
        ,SLOT(MainWinPlot_PlotPosConfidenceAppendData(uint32_t, uint8_t, uint8_t) ) );


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_PlotPosConfidenceReplot() )
        ,this
        ,SLOT(MainWinPlot_PlotPosConfidenceReplot() ));




    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_DebugTable_InsertDataRow(uint32_t,uint32_t,uint8_t,QString,QColor) )
        ,this
        ,SLOT(MainWin_DebugTable_InsertDataRow(uint32_t,uint32_t,uint8_t,QString,QColor) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_DebugTable_ScrollToBottom() )
        ,this
        ,SLOT(MainWin_DebugTable_ScrollToBottom() ) );



    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_RefreshErrorIndicatorView(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
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
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_CommunicationStatisticsUpdate(uint32_t,uint16_t,uint16_t,uint16_t,uint16_t) )
        ,this
        ,SLOT(MainWin_CommunicationStatisticsUpdate(uint32_t,uint16_t,uint16_t,uint16_t,uint16_t) ) );


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateErrorWeigthData(float,float,float,float,float,float,float,float,float,float,float,float))
        ,this
        ,SLOT(MainWin_UpdateNvmErrorWeigthData(float,float,float,float,float,float,float,float,float,float,float,float) ) );


    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateVehCfgData(float,uint32_t,uint32_t,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateNvM_VehCfgData(float, uint32_t, uint32_t,uint32_t) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdatePidData(float,float,float,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateNvM_PidData(float,float,float,uint32_t) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateRgAngleHndlrData(float,float,float,float,float,float,float,float,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateNvM_RgAngleHndlrData(float,float,float,float,float,float,float,float,uint32_t) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateMotorsFactors(uint32_t,uint32_t,uint32_t,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateMotorsFactors(uint32_t, uint32_t, uint32_t,uint32_t) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateEncoderCfgData(float,float) )
        ,this
        ,SLOT(MainWin_UpdateEncoderCfgData(float,float) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateSpeedProfileData(BluDataManager::BLU_NvM_SpdProfileData_t) )
        ,this
        ,SLOT(MainWin_UpdateSpeedProfileData(BluDataManager::BLU_NvM_SpdProfileData_t) ));

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_UpdateOrientation(float) )
        ,this
        ,SLOT(MainWin_DrawOrientationIndicator(float) ) );

}


/*********************************************************************************************************/
void MainWindow::MainWin_RefreshErrorIndicatorView( uint8_t S0,uint8_t S1,uint8_t S2,uint8_t S3,uint8_t S4,uint8_t S5,
                                                   uint8_t S6,uint8_t S7,uint8_t S8,uint8_t S9,uint8_t S10,uint8_t S11,
                                                   uint8_t RightLinePosConfif,uint8_t LeftLinePosConfif,
                                                   float PosError)
{


    QElapsedTimer timerIndView;
    timerIndView.start();

    if(ui->IndicatorVieEnableChBox->isChecked() )
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



        if(S0 > 170)
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

        if(S1 > 170)
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

        if(S2 > 170)
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

        if(S3 > 170)
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

        if(S4 > 170)
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


        if(S5 > 170)
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


        if(S6 > 170)
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

        if(S7 > 170)
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


        if(S8 > 170)
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


        if(S9 > 170)
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

        if(S10 > 170)
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

        if(S11 > 170)
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

    //        qDebug() << "MainWinRfrIndView TOOK: " << timerIndView.elapsed() << "milliseconds";

}
/*********************************************************************************************************/
void MainWindow::MainWin_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor )
{
    QColor QColorHelper = QColor(255,255,255); /*QColor default arg. value*/


    ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );

    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(ucTimeStamp) ));
    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem(QString::number(SyncId)));
    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,new QTableWidgetItem((QString)DecodedDataString.data() ) );


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
    BluInputDataProcessingWrapper.DebugTableScrollingBottonMutex.lock();
    BluInputDataProcessingWrapper.DebugTableScrollingBottomIsActivState = false;
    BluInputDataProcessingWrapper.DebugTableScrollingBottonMutex.unlock();
}


/*********************************************************************************************************/



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
    PlotTrvDistance.LfGraph_ClearData();
    PlotOrientation.LfGraph_ClearData();
    PlotLinePosConfidence.LfGraph_ClearData();
}

void MainWindow::MainWin_UpdateNvmErrorWeigthData(float ErrW1, float ErrW2, float ErrW3, float ErrW4, float ErrW5,
                                                    float ErrW6, float ErrW7, float ErrW8, float ErrW9, float ErrW10, float ErrW11, float ErrWM)
{
    NVM_ErrWeitghtsTabHolder[0] = ErrW1;
    NVM_ErrWeitghtsTabHolder[1] = ErrW2;
    NVM_ErrWeitghtsTabHolder[2] = ErrW3;
    NVM_ErrWeitghtsTabHolder[3] = ErrW4;
    NVM_ErrWeitghtsTabHolder[4] = ErrW5;
    NVM_ErrWeitghtsTabHolder[5] = ErrW6;
    NVM_ErrWeitghtsTabHolder[6] = ErrW7;
    NVM_ErrWeitghtsTabHolder[7] = ErrW8;
    NVM_ErrWeitghtsTabHolder[8] = ErrW9;
    NVM_ErrWeitghtsTabHolder[9] = ErrW10;
    NVM_ErrWeitghtsTabHolder[10] = ErrW11;
    NVM_ErrWeitghtsTabHolder[11] = ErrWM;

    NvM_ErrWeigthUpdateDelayTimer.setSingleShot(true);
    NvM_ErrWeigthUpdateDelayTimer.start(100);


}

void MainWindow::MainWin_UpdateNvM_PidData(float Kp, float Ki, float Kd, uint32_t ProbeTime)
{
    NvM_PID_Kp = Kp;
    NvM_PID_Ki = Ki;
    NvM_PID_Kd = Kd;
    NvM_ProbeTim = ProbeTime;

    NvM_PidDatahUpdateDelayTimer.setSingleShot(true);
    NvM_PidDatahUpdateDelayTimer.start(100);
}

void MainWindow::MainWin_UpdateNvM_RgAngleHndlrData(float rAgPidKp, float rAgPidKd,
                                                    float rAgBaseSpd, float rAgMaxYawRate,
                                                    float rAgBrakeSpeedTh, float rAgBrakingTime,
                                                    float rAgOriChange, float rAgOriChangeAfterBrake,
                                                    uint32_t rAgProbeTime)
{
    NvM_rAgHndlr_Kp = rAgPidKp;
    NvM_rAgHndlr_Kd = rAgPidKd;
    NvM_rAgBaseSpd = rAgBaseSpd;
    NVM_rAgMaxYawRate = rAgMaxYawRate;
    NVM_rAgBrakeSpeedTh = rAgBrakeSpeedTh;
    NVM_rAgBrakingTime = rAgBrakingTime;
    NVM_rAgOriChange = rAgOriChange;
    NVM_rAgOriChangeAfterBrake = rAgOriChangeAfterBrake;
    NvM_rAgProbeTime = rAgProbeTime;

    NvM_rAgDatahUpdateDelayTimer.setSingleShot(true);
    NvM_rAgDatahUpdateDelayTimer.start(100);
}

void MainWindow::MainWin_UpdateNvM_VehCfgData(float ExpectedAvSpd, uint32_t BlinkLedSt, uint32_t TryDetEndLin,uint32_t IrSensorIsEnabled)
{
    NvM_ExpectedAvSpeed = ExpectedAvSpd;
    NvM_BlinkLedSt = BlinkLedSt;
    NvM_TryDetEndLinSt = TryDetEndLin;
    NvM_isIrSensorEnabled = IrSensorIsEnabled;


    NvM_VehCfghUpdateDelayTimer.setSingleShot(true);
    NvM_VehCfghUpdateDelayTimer.start(100);
}

void MainWindow::MainWin_UpdateEncoderCfgData(float OneImpDist, float WheelBase)
{
    NVM_OneImpulsDistance = OneImpDist;
    NVM_WheelBase = WheelBase;

    NvM_EncoderCfgUpdateDelayTimer.setSingleShot(true);
    NvM_EncoderCfgUpdateDelayTimer.start(100);
}

void MainWindow::MainWin_UpdateSpeedProfileData(BluDataManager::BLU_NvM_SpdProfileData_t SpdProfileData)
{
    this->MW_NvM_SpdProfileData = SpdProfileData;
    NvM_SpeedProfileUpdateDelayTimer.setSingleShot(true);
    NvM_SpeedProfileUpdateDelayTimer.start(100);
}


void MainWindow::MainWin_UpdateMotorsFactors(uint32_t FacA_Lft, uint32_t FacA_Rgt,uint32_t FacB_Lft,uint32_t FacB_Rht)
{
    NvM_FacA_Lft = FacA_Lft;
    NvM_FacA_Rgt = FacA_Rgt;
    NvM_FacB_Lft = FacB_Lft;
    NvM_FacB_Rht = FacB_Rht;


    NvM_MotorsFactorsUpdateDelayTimer.setSingleShot(true);
    NvM_MotorsFactorsUpdateDelayTimer.start(100);
}


/*********************************************************************************************************/

void MainWindow::MainWinPlot_PlotPosErrReplot(void)
{
    int Index = ui->tabWidget_4->currentIndex();
    if(Index== 2)
    {
        PlotPosErr.LfGraph_UpdateReplot();
    }

    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.PosErrPlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotPidRegValReplot()
{
    int Index = ui->tabWidget_4->currentIndex();
    if(Index== 3)
    {
        PlotPidRegVal.LfGraph_UpdateReplot();
    }

    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.PidRegValPlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotPosErrAppendData(uint32_t FrameId, float PossErrValue)
{
    PlotPosErr.LfGraph_AppendData(FrameId,PossErrValue);
}

void MainWindow::MainWinPlot_PlotPidRegValAppendData(uint32_t FrameId, float PidRegVal)
{
    PlotPidRegVal.LfGraph_AppendData(FrameId,PidRegVal);
}


void MainWindow::MainWinPlot_PlotOrientationReplot()
{
    int Index = ui->PlotWidgetTab1->currentIndex();
    if(Index== 3)
    {
        PlotOrientation.LfGraph_UpdateReplot();
    }

    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.OrientationPlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotTrvDistanceReplot()
{
    int Index = ui->PlotWidgetTab1->currentIndex();
    if(Index== 2)
    {
        PlotTrvDistance.LfGraph_UpdateReplot();
    }

    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.TrvDistancePlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotPosConfidenceReplot()
{
    int Index = ui->tabWidget_4->currentIndex();
    if(Index== 4)
    {
        PlotLinePosConfidence.LfGraph_UpdateReplot();
    }

    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.LinePosConfPlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}


void MainWindow::MainWinPlot_PlotOrientationAppendData(uint32_t FrameId, float Orientation)
{
    PlotOrientation.LfGraph_AppendData(FrameId,Orientation);
}

void MainWindow::MainWinPlot_PlotTrvDistanceAppendData(uint32_t FrameId, float TrvDistance)
{
    PlotTrvDistance.LfGraph_AppendData(FrameId,TrvDistance);
}

void MainWindow::MainWinPlot_PlotPosConfidenceAppendData(uint32_t FrameId, uint8_t LeftPosConf, uint8_t RightPosConf)
{
    PlotLinePosConfidence.LfGraph_AppendData(FrameId,LeftPosConf, FrameId,RightPosConf);
}



void MainWindow::MainWinPlot_DrawMarkersAtDataIndexInfo(int DataIndex)
{
    static uint32_t CallCounter = 0;

    PlotMap.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotYawRate.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotSpd.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotPosErr.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotPidRegVal.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotTrvDistance.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotOrientation.LfGraph_DrawMarkersAtDataIndex(DataIndex);
    PlotLinePosConfidence.LfGraph_DrawMarkersAtDataIndex(DataIndex);

    float ClickedPosX = PlotMap.DataVector_X1.at(DataIndex);
    float ClickedPosY = PlotMap.DataVector_Y1.at(DataIndex);

    float ClickedOri = PlotOrientation.DataVector_Y1.at(DataIndex);
    MainWin_DrawOrientationIndicator(ClickedOri);

    float ClickedTrvDist = PlotTrvDistance.DataVector_Y1.at(DataIndex);
    uint8_t PlotLinePosConfidenceLeft = PlotLinePosConfidence.DataVector_Y1.at(DataIndex);
    uint8_t PlotLinePosConfidenceRight = PlotLinePosConfidence.DataVector_Y2.at(DataIndex);

    float ClickedYr = PlotYawRate.DataVector_Y1.at(DataIndex);
    float ClickedSpdL = PlotSpd.DataVector_Y1.at(DataIndex);
    float ClickedSpdR = PlotSpd.DataVector_Y2.at(DataIndex);
    float ClickedPosErr = PlotPosErr.DataVector_Y1.at(DataIndex);
    float ClickedPid = PlotPidRegVal.DataVector_Y1.at(DataIndex);

    QString ClickedPointString = QString("Clicked at point: PosX:%1  |PosY:%2  |Yr:%3  |SpdL:%4  |SpdR:%5  |PosErr:%6  |PidV:%7"
                                         " |Ori:%8 |TrvDist:%9 |LineConfL:%10  |LineConfR:%11")
                                     .arg(ClickedPosX).arg(ClickedPosY).arg(ClickedYr)
                                     .arg(ClickedSpdL).arg(ClickedSpdR).arg(ClickedPosErr)
                                     .arg(ClickedPid).arg(ClickedOri).arg(ClickedTrvDist)
                                     .arg(PlotLinePosConfidenceLeft).arg(PlotLinePosConfidenceRight);

    if(CallCounter % 2 == 0)
    {
        QVariant variant= QColor (35,35,45,255);
        QString colcode = variant.toString();
        ui->ClickedPointInfo_lb->setAutoFillBackground(true);
        ui->ClickedPointInfo_lb->setStyleSheet("QLabel { background-color :"+colcode+" ; color : white; }");
    }
    else{
        QVariant variant= QColor (25,45,45,255);
        QString colcode = variant.toString();
        ui->ClickedPointInfo_lb->setAutoFillBackground(true);
        ui->ClickedPointInfo_lb->setStyleSheet("QLabel { background-color :"+colcode+" ; color : white; }");
    }


    ui->ClickedPointInfo_lb->setText(ClickedPointString);

    CallCounter++;
}

void MainWindow::on_RemoveMarkers_pb_clicked()
{
    PlotMap.LfGraph_DrawMarkersAtDataIndex(0);
    PlotYawRate.LfGraph_DrawMarkersAtDataIndex(0);
    PlotSpd.LfGraph_DrawMarkersAtDataIndex(0);
    PlotPosErr.LfGraph_DrawMarkersAtDataIndex(0);
    PlotPidRegVal.LfGraph_DrawMarkersAtDataIndex(0);
    PlotTrvDistance.LfGraph_DrawMarkersAtDataIndex(0);
    PlotOrientation.LfGraph_DrawMarkersAtDataIndex(0);
    PlotLinePosConfidence.LfGraph_DrawMarkersAtDataIndex(0);
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
void MainWindow::MainWinPlot_PlotSpdAppendData(uint32_t FrameId, float SpdValueLeftWh,float SpdValueRightWh)
{
    PlotSpd.LfGraph_AppendData((float)FrameId,SpdValueLeftWh,(float)FrameId,SpdValueRightWh);
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotMapReplot(void)
{

//    QElapsedTimer timer;
//    timer.start();
    int Index = ui->MainTabWidget->currentIndex();
    if(Index== 1)
    {
        PlotMap.LfGraph_UpdateReplot();
    }
//    qDebug() << "MainWinPlot_PlotMapReplot TOOK: " << timer.elapsed() << "milliseconds";
    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.MapPlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotYawRateReplot(void)
{
//    qInfo() << this << "MainWinPlot_PlotYawRateReplot Thread:  " << QThread::currentThread();
    int Index = ui->PlotWidgetTab1->currentIndex();
    if(Index== 1)
    {
        PlotYawRate.LfGraph_UpdateReplot();
    }
    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.YawRatePlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}
/*********************************************************************************************************/
void MainWindow::MainWinPlot_PlotSpdReplot(void)
{
//    qInfo() << this << "MainWinPlot_PlotSpdReplot Thread:  " << QThread::currentThread();
    int Index = ui->tabWidget_4->currentIndex();
    if(Index== 1)
    {
        PlotSpd.LfGraph_UpdateReplot();
    }


    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BluInputDataProcessingWrapper.SpdPlotPlottingState = FALSE;
    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}
/*********************************************************************************************************/

/*
 * Drawing orientation indicator based on input given in radian
*/
void MainWindow::MainWin_DrawOrientationIndicator(float Orientation)
{
//    qDebug() << "MainWin_DrawOrientationIndicator Ori:" << Orientation;
    //    QElapsedTimer timerIndView;
    //    timerIndView.start();
    QPixmap pix(":/RobOri/RobOrientation/robotOriResource/RobotOriIndicator.png");
    QPainter paint(&pix);
    QTransform trans;
    trans.rotate( ((- 2 * M_PI) * 57.2957795) + (Orientation*57.2957795));

    ui->OrientationVectlabel->setPixmap(pix.transformed(trans) );

    //    qDebug() << "MainWin_DrawOrientationIndicator TOOK: " << timerIndView.elapsed() << "milliseconds";
}

void MainWindow::on_BLU_SimulatorStartButton_clicked()
{
    on_GeneralPlotDataClear_pb_clicked(); /*clear plot data...*/

    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {0};

    command[0] = (char)BluDataManager::BLU_SimulatorStart;

    for(int i=1; i<(BLU_SINGLE_TR_MESSAGE_SIZE-2); i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);


}
/*********************************************************************************************************/
void MainWindow::on_BLU_SimulatorSuspendButton_clicked()
{
    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2];
    command[0] = (char)BluDataManager::BLU_SimuAndTrueDataLoggingStop;

    for(int i=1; i<(BLU_SINGLE_TR_MESSAGE_SIZE-2); i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}

/*********************************************************************************************************/
void MainWindow::on_BLU_TrueLogStartButton_clicked()
{
    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2];
    command[0] = (char)BluDataManager::BLU_TrueBaseLoggingStart;

    for(int i=1; i<(BLU_SINGLE_TR_MESSAGE_SIZE-2); i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}

/*********************************************************************************************************/


void MainWindow::on_EnableBaseDataLogging_clicked(bool checked)
{
    if(true == checked)
    {
        BluInputDataProcessingWrapper.DebugTable_BaseDataLoggingState = true;
    }
}


void MainWindow::on_DebugTable_DisableBaseDataLogging_clicked(bool checked)
{
    if(true == checked)
    {
        BluInputDataProcessingWrapper.DebugTable_BaseDataLoggingState = false;
    }
}


void MainWindow::ReadNvMDataFromLineFollower()
{
    ui->ErrW1_Text->clear();
    ui->ErrW2_Text->clear();
    ui->ErrW3_Text->clear();
    ui->ErrW4_Text->clear();
    ui->ErrW5_Text->clear();
    ui->ErrW6_Text->clear();
    ui->ErrW7_Text->clear();
    ui->ErrW8_Text->clear();
    ui->ErrW9_Text->clear();
    ui->ErrW10_Text->clear();
    ui->ErrW11_Text->clear();
    ui->ErrWM_Text->clear();

    ui->ExpectedAvSpdText->clear();
    ui->PID_KP_text->clear();
    ui->PID_KI_text->clear();
    ui->PID_KD_text->clear();
    ui->ProbeTimeText->clear();

    ui->textRightAgKp->clear();
    ui->textRightAgKd->clear();
    ui->textRightAgBaseSpd->clear();
    ui->textRightAgMaxYr->clear();
    ui->textRightBrThr->clear();
    ui->textRightBrTim->clear();
    ui->textRightOriCh->clear();
    ui->textRightOriChAftBr->clear();
    ui->textRightAgPrTim->clear();

    ui->TextPwmToSpAFacL->clear();
    ui->TextPwmToSpAFacR->clear();
    ui->TextPwmToSpBFacL->clear();
    ui->TextPwmToSpBFacR->clear();

    ui->TextWheelBase->clear();
    ui->TextOneImpDist->clear();

    ui->SpdProfileBspd1->clear();
    ui->SpdProfileBspd2->clear();
    ui->SpdProfileBspd3->clear();
    ui->SpdProfileBspd4->clear();
    ui->SpdProfileBspd5->clear();
    ui->SpdProfileBspd6->clear();
    ui->SpdProfileBspd7->clear();
    ui->SpdProfileBspd8->clear();
    ui->SpdProfileBspd9->clear();
    ui->SpdProfileBspd10->clear();
    ui->SpdProfileBspd11->clear();
    ui->SpdProfileTrvD1->clear();
    ui->SpdProfileTrvD2->clear();
    ui->SpdProfileTrvD3->clear();
    ui->SpdProfileTrvD4->clear();
    ui->SpdProfileTrvD5->clear();
    ui->SpdProfileTrvD6->clear();
    ui->SpdProfileTrvD7->clear();
    ui->SpdProfileTrvD8->clear();
    ui->SpdProfileTrvD9->clear();
    ui->SpdProfileTrvD10->clear();
    ui->SpdProfileTrvD11->clear();

    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {'B'};
    command[0] = (char)BluDataManager::BLU_NvM_ErrWeigthSensorDataReq;

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

    command[0] = (char)BluDataManager::BLU_NvM_LinePidRegDataReq;
    Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

    command[0] = (char)BluDataManager::BLU_NvM_VehCfgReq;
    Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

    command[0] = (char)BluDataManager::BLU_NvM_RightAgHndlrDataReq;
    Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

    command[0] = (char)BluDataManager::BLU_NvM_MotorsFactorsReq;
    Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

    command[0] = (char)BluDataManager::BLU_NvM_EncoderModCfgReq;
    Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

    command[0] = (char)BluDataManager::BLU_NvM_SpdProfileReq;
    Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}


void MainWindow::on_ReadNvM_Button_clicked()
{
    ReadNvMDataFromLineFollower();
}

void MainWindow::on_UpdateNvM_Button_clicked()
{
    static uint8_t SyncID = 0;

    /*********************************************************************************************************/
    /*********************************************************************************************************/
    /*********************************************************************************************************/
    QString Err1Text = ui->ErrW1_Text->text();
    float Err1ValueFloat = Err1Text.toFloat();
    QString Err2Text = ui->ErrW2_Text->text();
    float Err2ValueFloat = Err2Text.toFloat();
    QString Err3Text = ui->ErrW3_Text->text();
    float Err3ValueFloat = Err3Text.toFloat();
    QString Err4Text = ui->ErrW4_Text->text();
    float Err4ValueFloat = Err4Text.toFloat();
    QString Err5Text = ui->ErrW5_Text->text();
    float Err5ValueFloat = Err5Text.toFloat();
    QString Err6Text = ui->ErrW6_Text->text();
    float Err6ValueFloat = Err6Text.toFloat();
    QString Err7Text = ui->ErrW7_Text->text();
    float Err7ValueFloat = Err7Text.toFloat();
    QString Err8Text = ui->ErrW8_Text->text();
    float Err8ValueFloat = Err8Text.toFloat();
    QString Err9Text = ui->ErrW9_Text->text();
    float Err9ValueFloat = Err9Text.toFloat();
    QString Err10Text = ui->ErrW10_Text->text();
    float Err10ValueFloat = Err10Text.toFloat();
    QString Err11Text = ui->ErrW11_Text->text();
    float Err11ValueFloat = Err11Text.toFloat();
    QString ErrMText = ui->ErrWM_Text->text();
    float ErrMValueFloat = ErrMText.toFloat();

    if(ErrMText.size() > 0) /*Update Nvm only if something is this text line*/
    {
        char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {'B'};
        QByteArray Helper;

        /***********************************************************************************/
        command[0] = (char)BluDataManager::BLU_NvM_ErrWeigthSensorData;
        command[1] = SyncID;
        std::memcpy(&command[2],  &Err1ValueFloat, sizeof(float));
        std::memcpy(&command[6],  &Err2ValueFloat, sizeof(float));
        std::memcpy(&command[10], &Err3ValueFloat, sizeof(float));
        std::memcpy(&command[14], &Err4ValueFloat, sizeof(float));
        std::memcpy(&command[18], &Err5ValueFloat, sizeof(float));
        std::memcpy(&command[22], &Err6ValueFloat, sizeof(float));
        std::memcpy(&command[26], &Err7ValueFloat, sizeof(float));
        std::memcpy(&command[30], &Err8ValueFloat, sizeof(float));
        std::memcpy(&command[34], &Err9ValueFloat, sizeof(float));
        std::memcpy(&command[38], &Err10ValueFloat, sizeof(float));
        std::memcpy(&command[42], &Err11ValueFloat, sizeof(float));
        std::memcpy(&command[46], &ErrMValueFloat, sizeof(float));
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/
        QString PID_KPtext = ui->PID_KP_text->text();
        float PID_KPfloat = PID_KPtext.toFloat();
        QString PID_KItext = ui->PID_KI_text->text();
        float PID_KIfloat = PID_KItext.toFloat();
        QString PID_KDtext = ui->PID_KD_text->text();
        float PID_KDfloat = PID_KDtext.toFloat();
        QString ProbeTimeText = ui->ProbeTimeText->text();
        uint32_t ProbeTimeInt = ProbeTimeText.toInt();

        ////    qDebug() << "PID_KDfloat" << PID_KDfloat << "ProbeTimeFloat" << ProbeTimeFloat;


        command[0] = (char)BluDataManager::BLU_NvM_LinePidRegData;
        command[1] = SyncID;
        std::memcpy(&command[2],  &PID_KPfloat, sizeof(float));
        std::memcpy(&command[6],  &PID_KIfloat, sizeof(float));
        std::memcpy(&command[10], &PID_KDfloat, sizeof(float));
        std::memcpy(&command[14], &ProbeTimeInt, sizeof(float));
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
        QThread::msleep(45);

        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/

        QString ExpectedAvSpdText = ui->ExpectedAvSpdText->text();
        float ExpectedAvSpdFloat = ExpectedAvSpdText.toFloat();

        uint32_t BlinkingLedsState = (uint32_t) ui->BlinkingLedsStateCheckBox->isChecked();
        uint32_t ThemeBlackType    = (uint32_t) ui->ThemeBlackTypeCheckBox->isChecked();
        uint32_t isIrSensorEnabled = (uint32_t) ui->IrSensorCheckBox->isChecked();

        ////    qDebug() << "BlinkingLedsState" << BlinkingLedsState;
        ////    qDebug() << "TryDetEndLineMark" << TryDetEndLineMark;


        command[0] = (char)BluDataManager::BLU_NvM_VehCfgData;
        command[1] = SyncID;
        std::memcpy(&command[2],  &ExpectedAvSpdFloat, sizeof(float));
        std::memcpy(&command[6],  &BlinkingLedsState, sizeof(uint32_t));
        std::memcpy(&command[10], &ThemeBlackType, sizeof(uint32_t));
        std::memcpy(&command[14], &isIrSensorEnabled, sizeof(uint32_t));
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
        QThread::msleep(45);

        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/
        QString rAgHndlrKpText = ui->textRightAgKp->text();
        float rAgHndlrKpFloat = rAgHndlrKpText.toFloat();

        QString rAgHndlrKdText = ui->textRightAgKd->text();
        float rAgHndlrKdFloat = rAgHndlrKdText.toFloat();

        QString rAgHndlrBaseSpdText = ui->textRightAgBaseSpd->text();
        float rAgHndlrBaseSpdFloat = rAgHndlrBaseSpdText.toFloat();

        QString rAgHndlrMaxYawRateText = ui->textRightAgMaxYr->text();
        float rAgHndlrMaxYawRateFloat = rAgHndlrMaxYawRateText.toFloat();


        QString textRightBrThrText = ui->textRightBrThr->text();
        float rAgHndlrBrThrFloat = textRightBrThrText.toFloat();


        QString textRightBrTimText = ui->textRightBrTim->text();
        float rAgHndlrBrTimFloat = textRightBrTimText.toFloat();


        QString textRightOriChText = ui->textRightOriCh->text();
        float rAgHndlrOriChFloat = textRightOriChText.toFloat();


        QString textRightOriChAftBrText = ui->textRightOriChAftBr->text();;
        float rAgHndlrOriChAftBrFloat = textRightOriChAftBrText.toFloat();


        QString rAgHndlrProbeTimText = ui->textRightAgPrTim->text();
        uint32_t rAgHndlrProbeTimU32 = rAgHndlrProbeTimText.toInt();

        command[0] = (char)BluDataManager::BLU_NvM_RightAgHndlrData;
        command[1] = SyncID;
        std::memcpy(&command[2],  &rAgHndlrKpFloat, sizeof(float));
        std::memcpy(&command[6],  &rAgHndlrKdFloat, sizeof(float));
        std::memcpy(&command[10], &rAgHndlrBaseSpdFloat, sizeof(float));
        std::memcpy(&command[14], &rAgHndlrMaxYawRateFloat, sizeof(float));
        std::memcpy(&command[18], &rAgHndlrBrThrFloat, sizeof(float));
        std::memcpy(&command[22], &rAgHndlrBrTimFloat, sizeof(float));
        std::memcpy(&command[26], &rAgHndlrOriChFloat, sizeof(float));
        std::memcpy(&command[30], &rAgHndlrOriChAftBrFloat, sizeof(float));
        std::memcpy(&command[34], &rAgHndlrProbeTimU32, sizeof(uint32_t));
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
        QThread::msleep(45);


        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/

        QString FacA_LftText = ui->TextPwmToSpAFacL->text();
        uint32_t FacA_LftU32 = FacA_LftText.toInt();
        QString FacA_RhtText = ui->TextPwmToSpAFacR->text();
        uint32_t FacA_RhtU32 = FacA_RhtText.toInt();
        QString FacB_LftText = ui->TextPwmToSpBFacL->text();
        uint32_t FacB_LftU32 = FacB_LftText.toInt();
        QString FacB_RhtText = ui->TextPwmToSpBFacR->text();
        uint32_t FacB_RhtU32 = FacB_RhtText.toInt();


        command[0] = (char)BluDataManager::BLU_NvM_MotorsFactorsData;
        command[1] = SyncID;
        std::memcpy(&command[2],  &FacA_LftU32, sizeof(uint32_t));
        std::memcpy(&command[6],  &FacA_RhtU32, sizeof(uint32_t));
        std::memcpy(&command[10], &FacB_LftU32, sizeof(uint32_t));
        std::memcpy(&command[14], &FacB_RhtU32, sizeof(uint32_t));
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
        QThread::msleep(45);
        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/

        QString OneImpDistText = ui->TextOneImpDist->text();
        float OneImpDistF32 = OneImpDistText.toFloat();
        QString WheelBaseText = ui->TextWheelBase->text();
        float WheelBaseF32 = WheelBaseText.toFloat();


        command[0] = (char)BluDataManager::BLU_NvM_EncoderModCfgData;
        command[1] = SyncID;
        std::memcpy(&command[2],  &OneImpDistF32, sizeof(float));
        std::memcpy(&command[6],  &WheelBaseF32, sizeof(float));

        //    qDebug() << "OneImpDistF32" << OneImpDistF32 << "WheelBaseF32" << WheelBaseF32;

        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
        QThread::msleep(45);


        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/
        {
            float SpdProfileBaseSpd[11];
            float SpdProfileTrvDist[11];

            QString SpdProfileBaseSpd1 = ui->SpdProfileBspd1->text();
            QString SpdProfileBaseSpd2 = ui->SpdProfileBspd2->text();
            QString SpdProfileBaseSpd3 = ui->SpdProfileBspd3->text();
            QString SpdProfileBaseSpd4 = ui->SpdProfileBspd4->text();
            QString SpdProfileBaseSpd5 = ui->SpdProfileBspd5->text();
            QString SpdProfileBaseSpd6 = ui->SpdProfileBspd6->text();
            QString SpdProfileBaseSpd7 = ui->SpdProfileBspd7->text();
            QString SpdProfileBaseSpd8 = ui->SpdProfileBspd8->text();
            QString SpdProfileBaseSpd9 = ui->SpdProfileBspd9->text();
            QString SpdProfileBaseSpd10 = ui->SpdProfileBspd10->text();
            QString SpdProfileBaseSpd11 = ui->SpdProfileBspd11->text();

            QString SpdProfileBaseTrvD1 = ui->SpdProfileTrvD1->text();
            QString SpdProfileBaseTrvD2 = ui->SpdProfileTrvD2->text();
            QString SpdProfileBaseTrvD3 = ui->SpdProfileTrvD3->text();
            QString SpdProfileBaseTrvD4 = ui->SpdProfileTrvD4->text();
            QString SpdProfileBaseTrvD5=  ui->SpdProfileTrvD5->text();
            QString SpdProfileBaseTrvD6 = ui->SpdProfileTrvD6->text();
            QString SpdProfileBaseTrvD7 = ui->SpdProfileTrvD7->text();
            QString SpdProfileBaseTrvD8 = ui->SpdProfileTrvD8->text();
            QString SpdProfileBaseTrvD9 = ui->SpdProfileTrvD9->text();
            QString SpdProfileBaseTrvD10 = ui->SpdProfileTrvD10->text();
            QString SpdProfileBaseTrvD11 = ui->SpdProfileTrvD11->text();

            SpdProfileBaseSpd[0] = SpdProfileBaseSpd1.toFloat();
            SpdProfileBaseSpd[1] = SpdProfileBaseSpd2.toFloat();
            SpdProfileBaseSpd[2] = SpdProfileBaseSpd3.toFloat();
            SpdProfileBaseSpd[3] = SpdProfileBaseSpd4.toFloat();
            SpdProfileBaseSpd[4] = SpdProfileBaseSpd5.toFloat();
            SpdProfileBaseSpd[5] = SpdProfileBaseSpd6.toFloat();
            SpdProfileBaseSpd[6] = SpdProfileBaseSpd7.toFloat();
            SpdProfileBaseSpd[7] = SpdProfileBaseSpd8.toFloat();
            SpdProfileBaseSpd[8] = SpdProfileBaseSpd9.toFloat();
            SpdProfileBaseSpd[9] = SpdProfileBaseSpd10.toFloat();
            SpdProfileBaseSpd[10] = SpdProfileBaseSpd11.toFloat();

            SpdProfileTrvDist[0] = SpdProfileBaseTrvD1.toFloat();
            SpdProfileTrvDist[1] = SpdProfileBaseTrvD2.toFloat();
            SpdProfileTrvDist[2] = SpdProfileBaseTrvD3.toFloat();
            SpdProfileTrvDist[3] = SpdProfileBaseTrvD4.toFloat();
            SpdProfileTrvDist[4] = SpdProfileBaseTrvD5.toFloat();
            SpdProfileTrvDist[5] = SpdProfileBaseTrvD6.toFloat();
            SpdProfileTrvDist[6] = SpdProfileBaseTrvD7.toFloat();
            SpdProfileTrvDist[7] = SpdProfileBaseTrvD8.toFloat();
            SpdProfileTrvDist[8] = SpdProfileBaseTrvD9.toFloat();
            SpdProfileTrvDist[9] = SpdProfileBaseTrvD10.toFloat();
            SpdProfileTrvDist[10] = SpdProfileBaseTrvD11.toFloat();

            uint32_t isSpdProfileEnabled = (uint32_t)ui->SpdProfileEnabled->isChecked();

            command[0] = (char)BluDataManager::BLU_NvM_SpdProfileData;
            command[1] = SyncID;
            std::memcpy(&command[2                          ],&isSpdProfileEnabled, sizeof(isSpdProfileEnabled));
            std::memcpy(&command[6                          ],&SpdProfileTrvDist, sizeof(SpdProfileTrvDist ));
            std::memcpy(&command[6+sizeof(SpdProfileTrvDist)],&SpdProfileBaseSpd, sizeof(SpdProfileBaseSpd));

            Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
            Helper.append("\n\r");
            BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
            QThread::msleep(45);
        }

        /*********************************************************************************************************/
        /*********************************************************************************************************/
        /*********************************************************************************************************/


        SyncID++;
    }

    ReadNvMDataFromLineFollower();
}

void MainWindow::NvM_ErrWeigthUpdateDelayTimerTimeout()
{
        ui->ErrW1_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[0],'f',2) );
        ui->ErrW2_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[1],'f',2) );
        ui->ErrW3_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[2],'f',2) );
        ui->ErrW4_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[3],'f',2) );
        ui->ErrW5_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[4],'f',2) );
        ui->ErrW6_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[5],'f',2) );
        ui->ErrW7_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[6],'f',2) );
        ui->ErrW8_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[7],'f',2) );
        ui->ErrW9_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[8],'f',2) );
        ui->ErrW10_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[9],'f',2) );
        ui->ErrW11_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[10],'f',2) );
        ui->ErrWM_Text->setText(QString::number(NVM_ErrWeitghtsTabHolder[11],'f',2) );
}

void MainWindow::NvM_PidDatahUpdateDelayTimerTimeout()
{
    ui->PID_KP_text->setText(QString::number(NvM_PID_Kp,'f',2) );
    ui->PID_KI_text->setText(QString::number(NvM_PID_Ki,'f',2) );
    ui->PID_KD_text->setText(QString::number(NvM_PID_Kd,'f',2) );
    ui->ProbeTimeText->setText(QString::number(NvM_ProbeTim,10) );
}

void MainWindow::NvM_rAgDatahUpdateDelayTimerTimeout()
{
    ui->textRightAgKp->setText(QString::number(NvM_rAgHndlr_Kp,'f',2));
    ui->textRightAgKd->setText(QString::number(NvM_rAgHndlr_Kd,'f',2));
    ui->textRightAgBaseSpd->setText(QString::number(NvM_rAgBaseSpd,'f',2));
    ui->textRightAgMaxYr->setText(QString::number(NVM_rAgMaxYawRate,'f',2));
    ui->textRightBrThr->setText(QString::number(NVM_rAgBrakeSpeedTh,'f',2));
    uint32_t brakingTimeInt = NVM_rAgBrakingTime; /* I'm lazy :) */
    ui->textRightBrTim->setText(QString::number(brakingTimeInt,10));
    ui->textRightOriCh->setText(QString::number(NVM_rAgOriChange,'f',2));
    ui->textRightOriChAftBr->setText(QString::number(NVM_rAgOriChangeAfterBrake,'f',2));
    ui->textRightAgPrTim->setText(QString::number(NvM_rAgProbeTime,10));
}

void MainWindow::NvM_VehCfgDataUpdateDelayTimerTimeout()
{
    ui->ExpectedAvSpdText->setText(QString::number(NvM_ExpectedAvSpeed,'f',2) );

//    qDebug() << "NvM_BlinkLedSt" << NvM_BlinkLedSt;
//    qDebug() << "NvM_TryDetEndLinSt" << NvM_TryDetEndLinSt;
    (NvM_isIrSensorEnabled== 0)?  ui->IrSensorCheckBox->setChecked(false) : ui->IrSensorCheckBox->setChecked(true);
    (NvM_TryDetEndLinSt == 0)  ?  ui->ThemeBlackTypeCheckBox->setChecked(false) : ui->ThemeBlackTypeCheckBox->setChecked(true);
    (NvM_BlinkLedSt == 0)      ?  ui->BlinkingLedsStateCheckBox->setChecked(false) : ui->BlinkingLedsStateCheckBox->setChecked(true);
}

void MainWindow::NvM_MotorsFactorsDataUpdateDelayTimerTimeout()
{
    ui->TextPwmToSpAFacL->setText(QString::number(NvM_FacA_Lft,10) );
    ui->TextPwmToSpAFacR->setText(QString::number(NvM_FacA_Rgt,10) );
    ui->TextPwmToSpBFacL->setText(QString::number(NvM_FacB_Lft,10) );
    ui->TextPwmToSpBFacR->setText(QString::number(NvM_FacB_Rht,10) );
}

void MainWindow::NvM_EncodersConfigDataUpdateDelayTimerTimeout()
{
    ui->TextWheelBase->setText(QString::number(NVM_WheelBase,'f',5) );
    ui->TextOneImpDist->setText(QString::number(NVM_OneImpulsDistance,'f',8) );
}

void MainWindow::NvM_ProfileSpeedConfigDataUpdateDelayTimerTimeout()
{
    (MW_NvM_SpdProfileData.EnabledFlag == 0) ? ui->SpdProfileEnabled->setChecked(false) : ui->SpdProfileEnabled->setChecked(true);

    ui->SpdProfileBspd1->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[0],'f',2) );
    ui->SpdProfileBspd2->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[1],'f',2) );
    ui->SpdProfileBspd3->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[2],'f',2) );
    ui->SpdProfileBspd4->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[3],'f',2) );
    ui->SpdProfileBspd5->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[4],'f',2) );
    ui->SpdProfileBspd6->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[5],'f',2) );
    ui->SpdProfileBspd7->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[6],'f',2) );
    ui->SpdProfileBspd8->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[7],'f',2) );
    ui->SpdProfileBspd9->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[8],'f',2) );
    ui->SpdProfileBspd10->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[9],'f',2) );
    ui->SpdProfileBspd11->setText( QString::number(MW_NvM_SpdProfileData.BaseSpeedValue[10],'f',2) );

    ui->SpdProfileTrvD1->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[0],'f',2) );
    ui->SpdProfileTrvD2->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[1],'f',2) );
    ui->SpdProfileTrvD3->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[2],'f',2) );
    ui->SpdProfileTrvD4->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[3],'f',2) );
    ui->SpdProfileTrvD5->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[4],'f',2) );
    ui->SpdProfileTrvD6->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[5],'f',2) );
    ui->SpdProfileTrvD7->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[6],'f',2) );
    ui->SpdProfileTrvD8->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[7],'f',2) );
    ui->SpdProfileTrvD9->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[8],'f',2) );
    ui->SpdProfileTrvD10->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[9],'f',2) );
    ui->SpdProfileTrvD11->setText( QString::number(MW_NvM_SpdProfileData.TrvDistance[10],'f',2) );
}




void MainWindow::on_ClearLoggerButton_clicked()
{
        ui->DebugDataTable->clearContents();
        ui->DebugDataTable->setRowCount(0);
}

void MainWindow::on_BLU_RobotStop_Button_clicked()
{
        ui->BLU_TrueLogStartButton->setDisabled(false);
        ui->BLU_SimulatorSuspendButton->setDisabled(false);
        ui->BLU_TrueLogStartButton->setDisabled(false);
        ui->BLU_SimulatorStartButton->setDisabled(false);
        ui->BLU_SimulatorSuspendButton->setDisabled(false);

        ui->BLU_TrueLogStartButton->setDisabled(false);

        ui->BLU_RobotStart_Button->setDisabled(false);
//        ui->ReadNvM_Button->setDisabled(false);
        ui->UpdateNvM_Button->setDisabled(false);
        ui->BLU_DisconnectButton->setDisabled(false);

        char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {'B'};
        QByteArray Helper;

        command[0] = (char)BluDataManager::BLU_RobotStop;
        command[1] = 0; //SyncID
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}

void MainWindow::on_BLU_RobotStart_Button_clicked()
{
        on_GeneralPlotDataClear_pb_clicked(); /*clear plot data...*/

        on_BLU_SimulatorSuspendButton_clicked();
        on_BLU_TrueLogStartButton_clicked();


        ui->BLU_TrueLogStartButton->setDisabled(true);
        ui->BLU_SimulatorSuspendButton->setDisabled(true);
        ui->BLU_SimulatorStartButton->setDisabled(true);
        ui->BLU_SimulatorSuspendButton->setDisabled(true);
        ui->BLU_TrueLogStartButton->setDisabled(true);
        ui->BLU_RobotStart_Button->setDisabled(true);
//        ui->ReadNvM_Button->setDisabled(true);
//        ui->UpdateNvM_Button->setDisabled(true);
        ui->BLU_DisconnectButton->setDisabled(true);

        char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {'B'};
        QByteArray Helper;

        command[0] = (char)BluDataManager::BLU_RobotStart;
        command[1] = 0; //SyncID
        Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);        
}

void MainWindow::on_UpdateExpectedAvSpd_clicked()
{
        QString ExpectedAvSpdText = ui->ExpectedAvSpdText->text();
        float ExpectedAvSpdFloat = ExpectedAvSpdText.toFloat();

        uint32_t BlinkingLedsState = (uint32_t) ui->BlinkingLedsStateCheckBox->isChecked();
        uint32_t ThemeBlackType = (uint32_t) ui->ThemeBlackTypeCheckBox->isChecked();


        char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {'B'};
        QByteArray Helper;

        command[0] = (char)BluDataManager::BLU_NvM_VehCfgData;
        command[1] = 0; //SyncID
        std::memcpy(&command[2],  &ExpectedAvSpdFloat, sizeof(float));
        std::memcpy(&command[6],  &BlinkingLedsState, sizeof(uint32_t));
        std::memcpy(&command[10], &ThemeBlackType, sizeof(uint32_t));
        Helper = QByteArray::fromRawData(command,(BLU_SINGLE_TR_MESSAGE_SIZE-2));
        Helper.append("\n\r");
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);

}

void MainWindow::on_UpdateLfNameButton_clicked()
{
        char command[BLU_SINGLE_TR_MESSAGE_SIZE-2] = {0};
        QByteArray Helper;
        command[0] = (char)BluDataManager::BLU_SetNewRobotName;
        command[1] = 0; //SyncID
        QString NewLineFollowerName = ui->NewLfNameText->text();
        QByteArray Helper2 = NewLineFollowerName.toUtf8();
        Helper = QByteArray::fromRawData(command,2);
        Helper = Helper + Helper2;
        qsizetype SizeNameFrame = Helper.size();

        while(SizeNameFrame < BLU_SINGLE_TR_MESSAGE_SIZE)
        {
        /*Just to be sure i'll send exactly 100bytes*/
        Helper.append('\0');
        SizeNameFrame = Helper.size();
        }

//        qDebug() << Helper;
        BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}

void MainWindow::on_GeneraReplotAllPlots_pb_clicked()
{
    PlotPosErr.LfGraph_UpdateReplot();
    PlotPidRegVal.LfGraph_UpdateReplot();
    PlotYawRate.LfGraph_UpdateReplot();
    PlotSpd.LfGraph_UpdateReplot();
    PlotMap.LfGraph_UpdateReplot();
    PlotTrvDistance.LfGraph_UpdateReplot();
    PlotOrientation.LfGraph_UpdateReplot();
    PlotLinePosConfidence.LfGraph_UpdateReplot();
}

void MainWindow::on_SaveAppState_pb_clicked()
{
    QString filter = "LfProject (*.lfp) ;; All files (*)";
    QString desktopPath = QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::DesktopLocation));
    QString file_name = QFileDialog::getSaveFileName(this,"choose file to overwrite",desktopPath,filter);

    QFile saveFile(file_name);

    if (!saveFile.open(QIODevice::WriteOnly)) {
        qWarning("Couldn't open save file.");
        return;
    }

    QJsonObject object;

    {
        QJsonArray YawRate_Y_JsArr;
        for(int i=0; i< PlotYawRate.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotYawRate.DataVector_Y1.at(i);
            YawRate_Y_JsArr.append(pointObject);
        }
        object["YawRate_Y"] = YawRate_Y_JsArr;
    }

    {
        QJsonArray PlotSpdL_Y_JsArr;
        for(int i=0; i< PlotSpd.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotSpd.DataVector_Y1.at(i);
            PlotSpdL_Y_JsArr.append(pointObject);
        }
        object["PlotSpdL_Y"] = PlotSpdL_Y_JsArr;
    }

    {
        QJsonArray PlotSpdR_Y_JsArr;
        for(int i=0; i< PlotSpd.DataVector_Y2.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotSpd.DataVector_Y2.at(i);
            PlotSpdR_Y_JsArr.append(pointObject);
        }
        object["PlotSpdR_Y"] = PlotSpdR_Y_JsArr;
    }

    {
        QJsonArray PlotPosErr_Y_JsArr;
        for(int i=0; i< PlotPosErr.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotPosErr.DataVector_Y1.at(i);
            PlotPosErr_Y_JsArr.append(pointObject);
        }
        object["PlotPosErr_Y"] = PlotPosErr_Y_JsArr;
    }

    {
        QJsonArray PlotPidRegVal_Y_JsArr;
        for(int i=0; i< PlotPidRegVal.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotPidRegVal.DataVector_Y1.at(i);
            PlotPidRegVal_Y_JsArr.append(pointObject);
        }
        object["PlotPidRegVal_Y"] = PlotPidRegVal_Y_JsArr;
    }

    {
        QJsonArray PlotTrvDistance_Y_JsArr;
        for(int i=0; i< PlotTrvDistance.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotTrvDistance.DataVector_Y1.at(i);
            PlotTrvDistance_Y_JsArr.append(pointObject);
        }
        object["PlotTrvDistance_Y"] = PlotTrvDistance_Y_JsArr;
    }

    {
        QJsonArray PlotOrientation_Y_JsArr;
        for(int i=0; i< PlotOrientation.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotOrientation.DataVector_Y1.at(i);
            PlotOrientation_Y_JsArr.append(pointObject);
        }
        object["PlotOrientation_Y"] = PlotOrientation_Y_JsArr;
    }

    {
        QJsonArray PlotLinePosConfidenceL_Y_JsArr;
        for(int i=0; i< PlotLinePosConfidence.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotLinePosConfidence.DataVector_Y1.at(i);
            PlotLinePosConfidenceL_Y_JsArr.append(pointObject);
        }
        object["PlotLinePosConfidenceL_Y"] = PlotLinePosConfidenceL_Y_JsArr;
    }

    {
        QJsonArray PlotLinePosConfidenceR_Y_JsArr;
        for(int i=0; i< PlotLinePosConfidence.DataVector_Y2.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotLinePosConfidence.DataVector_Y2.at(i);
            PlotLinePosConfidenceR_Y_JsArr.append(pointObject);
        }
        object["PlotLinePosConfidenceR_Y"] = PlotLinePosConfidenceR_Y_JsArr;
    }
    {
        QJsonArray PlotMapPosX_JsArr;
        for(int i=0; i< PlotMap.DataVector_X1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["X"]=PlotMap.DataVector_X1.at(i);
            PlotMapPosX_JsArr.append(pointObject);
        }
        object["PlotMap_PosX"] = PlotMapPosX_JsArr;
    }
    {
        QJsonArray PlotMapPosY_JsArr;
        for(int i=0; i< PlotMap.DataVector_Y1.size(); i++)
        {
            QJsonObject pointObject;
            pointObject["Y"]=PlotMap.DataVector_Y1.at(i);
            PlotMapPosY_JsArr.append(pointObject);
        }
        object["PlotMap_PosY"] = PlotMapPosY_JsArr;
    }
    {

        QJsonArray NvM_Data_JsArr;
        QJsonObject DataToStore;

        float SpdProfileBaseSpd[11];
        float SpdProfileTrvDist[11];

        QString SpdProfileBaseSpd1 = ui->SpdProfileBspd1->text();
        QString SpdProfileBaseSpd2 = ui->SpdProfileBspd2->text();
        QString SpdProfileBaseSpd3 = ui->SpdProfileBspd3->text();
        QString SpdProfileBaseSpd4 = ui->SpdProfileBspd4->text();
        QString SpdProfileBaseSpd5 = ui->SpdProfileBspd5->text();
        QString SpdProfileBaseSpd6 = ui->SpdProfileBspd6->text();
        QString SpdProfileBaseSpd7 = ui->SpdProfileBspd7->text();
        QString SpdProfileBaseSpd8 = ui->SpdProfileBspd8->text();
        QString SpdProfileBaseSpd9 = ui->SpdProfileBspd9->text();
        QString SpdProfileBaseSpd10 = ui->SpdProfileBspd10->text();
        QString SpdProfileBaseSpd11 = ui->SpdProfileBspd11->text();

        QString SpdProfileBaseTrvD1 = ui->SpdProfileTrvD1->text();
        QString SpdProfileBaseTrvD2 = ui->SpdProfileTrvD2->text();
        QString SpdProfileBaseTrvD3 = ui->SpdProfileTrvD3->text();
        QString SpdProfileBaseTrvD4 = ui->SpdProfileTrvD4->text();
        QString SpdProfileBaseTrvD5=  ui->SpdProfileTrvD5->text();
        QString SpdProfileBaseTrvD6 = ui->SpdProfileTrvD6->text();
        QString SpdProfileBaseTrvD7 = ui->SpdProfileTrvD7->text();
        QString SpdProfileBaseTrvD8 = ui->SpdProfileTrvD8->text();
        QString SpdProfileBaseTrvD9 = ui->SpdProfileTrvD9->text();
        QString SpdProfileBaseTrvD10 = ui->SpdProfileTrvD10->text();
        QString SpdProfileBaseTrvD11 = ui->SpdProfileTrvD11->text();
        SpdProfileBaseSpd[0] = SpdProfileBaseSpd1.toFloat();
        SpdProfileBaseSpd[1] = SpdProfileBaseSpd2.toFloat();
        SpdProfileBaseSpd[2] = SpdProfileBaseSpd3.toFloat();
        SpdProfileBaseSpd[3] = SpdProfileBaseSpd4.toFloat();
        SpdProfileBaseSpd[4] = SpdProfileBaseSpd5.toFloat();
        SpdProfileBaseSpd[5] = SpdProfileBaseSpd6.toFloat();
        SpdProfileBaseSpd[6] = SpdProfileBaseSpd7.toFloat();
        SpdProfileBaseSpd[7] = SpdProfileBaseSpd8.toFloat();
        SpdProfileBaseSpd[8] = SpdProfileBaseSpd9.toFloat();
        SpdProfileBaseSpd[9] = SpdProfileBaseSpd10.toFloat();
        SpdProfileBaseSpd[10] = SpdProfileBaseSpd11.toFloat();
        SpdProfileTrvDist[0] = SpdProfileBaseTrvD1.toFloat();
        SpdProfileTrvDist[1] = SpdProfileBaseTrvD2.toFloat();
        SpdProfileTrvDist[2] = SpdProfileBaseTrvD3.toFloat();
        SpdProfileTrvDist[3] = SpdProfileBaseTrvD4.toFloat();
        SpdProfileTrvDist[4] = SpdProfileBaseTrvD5.toFloat();
        SpdProfileTrvDist[5] = SpdProfileBaseTrvD6.toFloat();
        SpdProfileTrvDist[6] = SpdProfileBaseTrvD7.toFloat();
        SpdProfileTrvDist[7] = SpdProfileBaseTrvD8.toFloat();
        SpdProfileTrvDist[8] = SpdProfileBaseTrvD9.toFloat();
        SpdProfileTrvDist[9] = SpdProfileBaseTrvD10.toFloat();
        SpdProfileTrvDist[10] = SpdProfileBaseTrvD11.toFloat();
        uint32_t isSpdProfileEnabled = (uint32_t)ui->SpdProfileEnabled->isChecked();
        DataToStore["SpdProfileBaseSpd_0"]=SpdProfileBaseSpd[0];
        DataToStore["SpdProfileBaseSpd_1"]=SpdProfileBaseSpd[1];
        DataToStore["SpdProfileBaseSpd_2"]=SpdProfileBaseSpd[2];
        DataToStore["SpdProfileBaseSpd_3"]=SpdProfileBaseSpd[3];
        DataToStore["SpdProfileBaseSpd_4"]=SpdProfileBaseSpd[4];
        DataToStore["SpdProfileBaseSpd_5"]=SpdProfileBaseSpd[5];
        DataToStore["SpdProfileBaseSpd_6"]=SpdProfileBaseSpd[6];
        DataToStore["SpdProfileBaseSpd_7"]=SpdProfileBaseSpd[7];
        DataToStore["SpdProfileBaseSpd_8"]=SpdProfileBaseSpd[8];
        DataToStore["SpdProfileBaseSpd_9"]=SpdProfileBaseSpd[9];
        DataToStore["SpdProfileBaseSpd_10"]=SpdProfileBaseSpd[10];

        DataToStore["SpdProfileTrvDist_0"]=SpdProfileTrvDist[0];
        DataToStore["SpdProfileTrvDist_1"]=SpdProfileTrvDist[1];
        DataToStore["SpdProfileTrvDist_2"]=SpdProfileTrvDist[2];
        DataToStore["SpdProfileTrvDist_3"]=SpdProfileTrvDist[3];
        DataToStore["SpdProfileTrvDist_4"]=SpdProfileTrvDist[4];
        DataToStore["SpdProfileTrvDist_5"]=SpdProfileTrvDist[5];
        DataToStore["SpdProfileTrvDist_6"]=SpdProfileTrvDist[6];
        DataToStore["SpdProfileTrvDist_7"]=SpdProfileTrvDist[7];
        DataToStore["SpdProfileTrvDist_8"]=SpdProfileTrvDist[8];
        DataToStore["SpdProfileTrvDist_9"]=SpdProfileTrvDist[9];
        DataToStore["SpdProfileTrvDist_10"]=SpdProfileTrvDist[10];

        DataToStore["isSpdProfileEnabled"]=(int)isSpdProfileEnabled;


        QString Err1Text = ui->ErrW1_Text->text();
        float Err1ValueFloat = Err1Text.toFloat();
        QString Err2Text = ui->ErrW2_Text->text();
        float Err2ValueFloat = Err2Text.toFloat();
        QString Err3Text = ui->ErrW3_Text->text();
        float Err3ValueFloat = Err3Text.toFloat();
        QString Err4Text = ui->ErrW4_Text->text();
        float Err4ValueFloat = Err4Text.toFloat();
        QString Err5Text = ui->ErrW5_Text->text();
        float Err5ValueFloat = Err5Text.toFloat();
        QString Err6Text = ui->ErrW6_Text->text();
        float Err6ValueFloat = Err6Text.toFloat();
        QString Err7Text = ui->ErrW7_Text->text();
        float Err7ValueFloat = Err7Text.toFloat();
        QString Err8Text = ui->ErrW8_Text->text();
        float Err8ValueFloat = Err8Text.toFloat();
        QString Err9Text = ui->ErrW9_Text->text();
        float Err9ValueFloat = Err9Text.toFloat();
        QString Err10Text = ui->ErrW10_Text->text();
        float Err10ValueFloat = Err10Text.toFloat();
        QString Err11Text = ui->ErrW11_Text->text();
        float Err11ValueFloat = Err11Text.toFloat();
        QString ErrMText = ui->ErrWM_Text->text();
        float ErrMValueFloat = ErrMText.toFloat();

        DataToStore["Err1ValueFloat"]= Err1ValueFloat;
        DataToStore["Err2ValueFloat"]= Err2ValueFloat;
        DataToStore["Err3ValueFloat"]= Err3ValueFloat;
        DataToStore["Err4ValueFloat"]= Err4ValueFloat;
        DataToStore["Err5ValueFloat"]= Err5ValueFloat;
        DataToStore["Err6ValueFloat"]= Err6ValueFloat;
        DataToStore["Err7ValueFloat"]= Err7ValueFloat;
        DataToStore["Err8ValueFloat"]= Err8ValueFloat;
        DataToStore["Err9ValueFloat"]= Err9ValueFloat;
        DataToStore["Err10ValueFloat"]= Err10ValueFloat;
        DataToStore["Err11ValueFloat"]= Err11ValueFloat;
        DataToStore["ErrMValueFloat"]= ErrMValueFloat;

        QString PID_KPtext = ui->PID_KP_text->text();
        float PID_KPfloat = PID_KPtext.toFloat();
        QString PID_KItext = ui->PID_KI_text->text();
        float PID_KIfloat = PID_KItext.toFloat();
        QString PID_KDtext = ui->PID_KD_text->text();
        float PID_KDfloat = PID_KDtext.toFloat();
        QString ProbeTimeText = ui->ProbeTimeText->text();
        uint32_t ProbeTimeInt = ProbeTimeText.toInt();
        QString ExpectedAvSpdText = ui->ExpectedAvSpdText->text();
        float ExpectedAvSpdFloat = ExpectedAvSpdText.toFloat();

        DataToStore["PID_KPfloat"]= PID_KPfloat;
        DataToStore["PID_KIfloat"]= PID_KIfloat;
        DataToStore["PID_KDfloat"]= PID_KDfloat;
        DataToStore["ProbeTimeInt"]= (int)ProbeTimeInt;
        DataToStore["ExpectedAvSpdFloat"]= ExpectedAvSpdFloat;


        uint32_t BlinkingLedsState = (uint32_t) ui->BlinkingLedsStateCheckBox->isChecked();
        uint32_t TryDetEndLineMark = (uint32_t) ui->ThemeBlackTypeCheckBox->isChecked();
        uint32_t isIrSensorEnabled = (uint32_t) ui->IrSensorCheckBox->isChecked();

        DataToStore["BlinkingLedsState"]= (int)BlinkingLedsState;
        DataToStore["TryDetEndLineMark"]= (int)TryDetEndLineMark;
        DataToStore["isIrSensorEnabled"]= (int)isIrSensorEnabled;


        QString rAgHndlrKpText = ui->textRightAgKp->text();
        float rAgHndlrKpFloat = rAgHndlrKpText.toFloat();
        DataToStore["rAgHndlrKpFloat"]= rAgHndlrKpFloat;

        QString rAgHndlrKdText = ui->textRightAgKd->text();
        float rAgHndlrKdFloat = rAgHndlrKdText.toFloat();
        DataToStore["rAgHndlrKdFloat"]= rAgHndlrKdFloat;

        QString rAgHndlrBaseSpdText = ui->textRightAgBaseSpd->text();
        float rAgHndlrBaseSpdFloat = rAgHndlrBaseSpdText.toFloat();
        DataToStore["rAgHndlrBaseSpdFloat"]= rAgHndlrBaseSpdFloat;

        QString rAgHndlrMaxYawRateText = ui->textRightAgMaxYr->text();
        float rAgHndlrMaxYawRateFloat = rAgHndlrMaxYawRateText.toFloat();
        DataToStore["rAgHndlrMaxYawRateFloat"]= rAgHndlrMaxYawRateFloat;

        QString textRightBrThrText = ui->textRightBrThr->text();
        float rAgHndlrBrThrFloat = textRightBrThrText.toFloat();
        DataToStore["rAgHndlrBrThrFloat"]= rAgHndlrBrThrFloat;

        QString textRightBrTimText = ui->textRightBrTim->text();
        float rAgHndlrBrTimFloat = textRightBrTimText.toFloat();
        DataToStore["rAgHndlrBrTimFloat"]= rAgHndlrBrTimFloat;

        QString textRightOriChText = ui->textRightOriCh->text();
        float rAgHndlrOriChFloat = textRightOriChText.toFloat();
        DataToStore["rAgHndlrOriChFloat"]= rAgHndlrOriChFloat;

        QString textRightOriChAftBrText = ui->textRightOriChAftBr->text();
        float rAgHndlrOriChAftBrFloat = textRightOriChAftBrText.toFloat();
        DataToStore["rAgHndlrOriChAftBrFloat"]= rAgHndlrOriChAftBrFloat;

        QString rAgHndlrProbeTimText = ui->textRightAgPrTim->text();
        uint32_t rAgHndlrProbeTimU32 = rAgHndlrProbeTimText.toInt();
        DataToStore["rAgHndlrProbeTimU32"]= (int)rAgHndlrProbeTimU32;

        QString FacA_LftText = ui->TextPwmToSpAFacL->text();
        uint32_t FacA_LftU32 = FacA_LftText.toInt();
        QString FacA_RhtText = ui->TextPwmToSpAFacR->text();
        uint32_t FacA_RhtU32 = FacA_RhtText.toInt();
        QString FacB_LftText = ui->TextPwmToSpBFacL->text();
        uint32_t FacB_LftU32 = FacB_LftText.toInt();
        QString FacB_RhtText = ui->TextPwmToSpBFacR->text();
        uint32_t FacB_RhtU32 = FacB_RhtText.toInt();

        DataToStore["FacA_LftU32"]= (int)FacA_LftU32;
        DataToStore["FacA_RhtU32"]= (int)FacA_RhtU32;
        DataToStore["FacB_LftU32"]= (int)FacB_LftU32;
        DataToStore["FacB_RhtU32"]= (int)FacB_RhtU32;



        QString OneImpDistText = ui->TextOneImpDist->text();
        float OneImpDistF32 = OneImpDistText.toFloat();
        QString WheelBaseText = ui->TextWheelBase->text();
        float WheelBaseF32 = WheelBaseText.toFloat();

        DataToStore["OneImpDistF32"]= OneImpDistF32;
        DataToStore["WheelBaseF32"]= WheelBaseF32;



        NvM_Data_JsArr.append(DataToStore);


        object["NvmData"] = NvM_Data_JsArr;
    }
    {
        QJsonArray DebugDataTable_JsArr;

        for(int i=0; i< ui->DebugDataTable->rowCount(); i++)
        {
            QTableWidgetItem *Cell_rowI_Cell0_sysTime      = (ui->DebugDataTable->item(i,0) );
            QTableWidgetItem *Cell_rowI_Cell1_ucTimeStamp  = (ui->DebugDataTable->item(i,1) );
            QTableWidgetItem *Cell_rowI_Cell2_FrameCounter = (ui->DebugDataTable->item(i,2) );
            QTableWidgetItem *Cell_rowI_Cell3_SyncId       = (ui->DebugDataTable->item(i,3) );
            QTableWidgetItem *Cell_rowI_Cell4_Data         = (ui->DebugDataTable->item(i,4) );

            QJsonObject RowData;
            RowData["sysTime"]= Cell_rowI_Cell0_sysTime ? Cell_rowI_Cell0_sysTime->text() : 0;
            RowData["ucTimeStamp"]= Cell_rowI_Cell1_ucTimeStamp ? Cell_rowI_Cell1_ucTimeStamp->text() : 0;
            RowData["FrameCounter"]= Cell_rowI_Cell2_FrameCounter ? Cell_rowI_Cell2_FrameCounter->text() : 0;
            RowData["SyncId"]= Cell_rowI_Cell3_SyncId ? Cell_rowI_Cell3_SyncId->text() : 0;
            RowData["Data"]= Cell_rowI_Cell4_Data ? Cell_rowI_Cell4_Data->text() : 0;

            DebugDataTable_JsArr.append(RowData);
        }

        object["mDebugDataLogger"] = DebugDataTable_JsArr;
    }
    saveFile.write(QJsonDocument(object).toJson() );

    QString MessageBoxString = QString("Sucessfully saved \n NvM Data \n Plot Data \n Logger Table Data \n to file:\n %1").arg(file_name);
    QMessageBox::about(this,"Success!", MessageBoxString);
}

void MainWindow::LoadDataLineFollowerProjecrOrJson(QString FilePath)
{
    QFile loadFile(FilePath);

    if (!loadFile.open(QIODevice::ReadOnly)) {
        qWarning("Couldn't open load file.");
        return;
    }

    QByteArray loadedData = loadFile.readAll();

    QJsonDocument loadDoc( QJsonDocument::fromJson(loadedData));

    if (loadDoc.object().contains("NvmData") && loadDoc.object()["NvmData"].isArray())
    {
        QJsonArray NvmData = loadDoc.object()["NvmData"].toArray();
        QJsonObject NvmDataObject = NvmData.at(0).toObject();

        ui->SpdProfileTrvD1->setText( QString::number(NvmDataObject["SpdProfileTrvDist_0"].toDouble(),'f',2) );
        ui->SpdProfileTrvD2->setText( QString::number(NvmDataObject["SpdProfileTrvDist_1"].toDouble(),'f',2) );
        ui->SpdProfileTrvD3->setText( QString::number(NvmDataObject["SpdProfileTrvDist_2"].toDouble(),'f',2) );
        ui->SpdProfileTrvD4->setText( QString::number(NvmDataObject["SpdProfileTrvDist_3"].toDouble(),'f',2) );
        ui->SpdProfileTrvD5->setText( QString::number(NvmDataObject["SpdProfileTrvDist_4"].toDouble(),'f',2) );
        ui->SpdProfileTrvD6->setText( QString::number(NvmDataObject["SpdProfileTrvDist_5"].toDouble(),'f',2) );
        ui->SpdProfileTrvD7->setText( QString::number(NvmDataObject["SpdProfileTrvDist_6"].toDouble(),'f',2) );
        ui->SpdProfileTrvD8->setText( QString::number(NvmDataObject["SpdProfileTrvDist_7"].toDouble(),'f',2) );
        ui->SpdProfileTrvD9->setText( QString::number(NvmDataObject["SpdProfileTrvDist_8"].toDouble(),'f',2) );
        ui->SpdProfileTrvD10->setText( QString::number(NvmDataObject["SpdProfileTrvDist_9"].toDouble(),'f',2) );
        ui->SpdProfileTrvD11->setText( QString::number(NvmDataObject["SpdProfileTrvDist_10"].toDouble(),'f',2) );

        ui->SpdProfileBspd1->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_0"].toDouble(),'f',2) );
        ui->SpdProfileBspd2->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_1"].toDouble(),'f',2) );
        ui->SpdProfileBspd3->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_2"].toDouble(),'f',2) );
        ui->SpdProfileBspd4->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_3"].toDouble(),'f',2) );
        ui->SpdProfileBspd5->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_4"].toDouble(),'f',2) );
        ui->SpdProfileBspd6->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_5"].toDouble(),'f',2) );
        ui->SpdProfileBspd7->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_6"].toDouble(),'f',2) );
        ui->SpdProfileBspd8->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_7"].toDouble(),'f',2) );
        ui->SpdProfileBspd9->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_8"].toDouble(),'f',2) );
        ui->SpdProfileBspd10->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_9"].toDouble(),'f',2) );
        ui->SpdProfileBspd11->setText( QString::number(NvmDataObject["SpdProfileBaseSpd_10"].toDouble(),'f',2) );

        ui->ErrW1_Text->setText(QString::number(NvmDataObject["Err1ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW2_Text->setText(QString::number(NvmDataObject["Err2ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW3_Text->setText(QString::number(NvmDataObject["Err3ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW4_Text->setText(QString::number(NvmDataObject["Err4ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW5_Text->setText(QString::number(NvmDataObject["Err5ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW6_Text->setText(QString::number(NvmDataObject["Err6ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW7_Text->setText(QString::number(NvmDataObject["Err7ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW8_Text->setText(QString::number(NvmDataObject["Err8ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW9_Text->setText(QString::number(NvmDataObject["Err9ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW10_Text->setText(QString::number(NvmDataObject["Err10ValueFloat"].toDouble() ,'f',2) );
        ui->ErrW11_Text->setText(QString::number(NvmDataObject["Err11ValueFloat"].toDouble() ,'f',2) );
        ui->ErrWM_Text->setText(QString::number(NvmDataObject["ErrMValueFloat"].toDouble() ,'f',2) );

        ui->PID_KP_text->setText(QString::number(NvmDataObject["PID_KPfloat"].toDouble() ,'f',2) );
        ui->PID_KI_text->setText(QString::number(NvmDataObject["PID_KIfloat"].toDouble() ,'f',2) );
        ui->PID_KD_text->setText(QString::number(NvmDataObject["PID_KDfloat"].toDouble() ,'f',2) );
        ui->ProbeTimeText->setText(QString::number(NvmDataObject["ProbeTimeInt"].toInt() ) );
        ui->ExpectedAvSpdText->setText(QString::number(NvmDataObject["ExpectedAvSpdFloat"].toDouble() ,'f',2) );

        (NvmDataObject["BlinkingLedsState"].toInt() == 0)   ? ui->BlinkingLedsStateCheckBox->setChecked(false) : ui->BlinkingLedsStateCheckBox->setChecked(true);
        (NvmDataObject["TryDetEndLineMark"].toInt() == 0)   ? ui->ThemeBlackTypeCheckBox->setChecked(false) : ui->ThemeBlackTypeCheckBox->setChecked(true);
        (NvmDataObject["isIrSensorEnabled"].toInt() == 0)   ? ui->IrSensorCheckBox->setChecked(false) : ui->IrSensorCheckBox->setChecked(true);
        (NvmDataObject["isSpdProfileEnabled"].toInt() == 0) ? ui->SpdProfileEnabled->setChecked(false) : ui->SpdProfileEnabled->setChecked(true);

        ui->textRightAgKp->setText(QString::number(NvmDataObject["rAgHndlrKpFloat"].toDouble() ,'f',2) );
        ui->textRightAgKd->setText(QString::number(NvmDataObject["rAgHndlrKdFloat"].toDouble() ,'f',2) );
        ui->textRightAgBaseSpd->setText(QString::number(NvmDataObject["rAgHndlrBaseSpdFloat"].toDouble() ,'f',2) );
        ui->textRightAgMaxYr->setText(QString::number(NvmDataObject["rAgHndlrMaxYawRateFloat"].toDouble() ,'f',2) );
        ui->textRightBrThr->setText(QString::number(NvmDataObject["rAgHndlrBrThrFloat"].toDouble() ,'f',2) );
        ui->textRightBrTim->setText(QString::number(NvmDataObject["rAgHndlrBrTimFloat"].toDouble() ,'f',2) );
        ui->textRightOriCh->setText(QString::number(NvmDataObject["rAgHndlrOriChFloat"].toDouble() ,'f',2) );
        ui->textRightOriChAftBr->setText(QString::number(NvmDataObject["rAgHndlrOriChAftBrFloat"].toDouble() ,'f',2) );
        ui->textRightAgPrTim->setText(QString::number(NvmDataObject["rAgHndlrProbeTimU32"].toInt() ) );

        ui->TextPwmToSpAFacL->setText(QString::number(NvmDataObject["FacA_LftU32"].toInt()) );
        ui->TextPwmToSpAFacR->setText(QString::number(NvmDataObject["FacA_RhtU32"].toInt()) );
        ui->TextPwmToSpBFacL->setText(QString::number(NvmDataObject["FacB_LftU32"].toInt()) );
        ui->TextPwmToSpBFacR->setText(QString::number(NvmDataObject["FacB_RhtU32"].toInt()) );

        ui->TextOneImpDist->setText(QString::number(NvmDataObject["OneImpDistF32"].toDouble() ,'f',7) );
        ui->TextWheelBase->setText(QString::number(NvmDataObject["WheelBaseF32"].toDouble() ,'f',7) );

    }

    if (loadDoc.object().contains("YawRate_Y") && loadDoc.object()["YawRate_Y"].isArray())
    {

        {
            QJsonArray YawRate_Yarr = loadDoc.object()["YawRate_Y"].toArray();
            PlotYawRate.DataVector_Y1.clear();
            PlotYawRate.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : YawRate_Yarr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                //                qDebug() << "LoadPlotDataTestYawRate DataPointValue: " << DataPoint;
                PlotYawRate.DataVector_X1.append(IterCounter);
                PlotYawRate.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotSpdL_Y_JsArr = loadDoc.object()["PlotSpdL_Y"].toArray();
            PlotSpd.DataVector_Y1.clear();
            PlotSpd.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotSpdL_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotSpd.DataVector_X1.append(IterCounter);
                PlotSpd.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotSpdR_Y_JsArr = loadDoc.object()["PlotSpdR_Y"].toArray();
            PlotSpd.DataVector_Y2.clear();
            PlotSpd.DataVector_X2.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotSpdR_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotSpd.DataVector_X2.append(IterCounter);
                PlotSpd.DataVector_Y2.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotPosErr_Y_JsArr = loadDoc.object()["PlotPosErr_Y"].toArray();
            PlotPosErr.DataVector_Y1.clear();
            PlotPosErr.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotPosErr_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotPosErr.DataVector_X1.append(IterCounter);
                PlotPosErr.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotPidRegVal_Y_JsArr = loadDoc.object()["PlotPidRegVal_Y"].toArray();
            PlotPidRegVal.DataVector_Y1.clear();
            PlotPidRegVal.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotPidRegVal_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotPidRegVal.DataVector_X1.append(IterCounter);
                PlotPidRegVal.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotTrvDistance_Y_JsArr = loadDoc.object()["PlotTrvDistance_Y"].toArray();
            PlotTrvDistance.DataVector_Y1.clear();
            PlotTrvDistance.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotTrvDistance_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotTrvDistance.DataVector_X1.append(IterCounter);
                PlotTrvDistance.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotOrientation_Y_JsArr = loadDoc.object()["PlotOrientation_Y"].toArray();
            PlotOrientation.DataVector_Y1.clear();
            PlotOrientation.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotOrientation_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotOrientation.DataVector_X1.append(IterCounter);
                PlotOrientation.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotLinePosConfidenceL_Y_JsArr = loadDoc.object()["PlotLinePosConfidenceL_Y"].toArray();
            PlotLinePosConfidence.DataVector_Y1.clear();
            PlotLinePosConfidence.DataVector_X1.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotLinePosConfidenceL_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotLinePosConfidence.DataVector_X1.append(IterCounter);
                PlotLinePosConfidence.DataVector_Y1.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotLinePosConfidenceR_Y_JsArr = loadDoc.object()["PlotLinePosConfidenceR_Y"].toArray();
            PlotLinePosConfidence.DataVector_Y2.clear();
            PlotLinePosConfidence.DataVector_X2.clear();
            int IterCounter = 0;
            for (const QJsonValue &v : PlotLinePosConfidenceR_Y_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotLinePosConfidence.DataVector_X2.append(IterCounter);
                PlotLinePosConfidence.DataVector_Y2.append(DataPoint);
                IterCounter++;
            }

        }
        {
            QJsonArray PlotMap_PosX_JsArr = loadDoc.object()["PlotMap_PosX"].toArray();
            PlotMap.DataVector_X1.clear();
            for (const QJsonValue &v : PlotMap_PosX_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["X"].toDouble();
                PlotMap.DataVector_X1.append(DataPoint);
            }

        }
        {
            QJsonArray PlotMap_PosY_JsArr = loadDoc.object()["PlotMap_PosY"].toArray();
            PlotMap.DataVector_Y1.clear();
            for (const QJsonValue &v : PlotMap_PosY_JsArr) {
                QJsonObject DataPoint_json = v.toObject();
                float DataPoint =  DataPoint_json["Y"].toDouble();
                PlotMap.DataVector_Y1.append(DataPoint);
            }

        }
        {
            QJsonArray DebugDataTable_JsArr = loadDoc.object()["mDebugDataLogger"].toArray();

            for (const QJsonValue &v : DebugDataTable_JsArr)
            {
                QJsonObject DataRow_json = v.toObject();

                QString sysTime = DataRow_json["sysTime"].toString();
                QString ucTimeStamp = DataRow_json["ucTimeStamp"].toString();
                QString FrameCounter = DataRow_json["FrameCounter"].toString();
                QString SyncId = DataRow_json["SyncId"].toString();
                QString DataString = DataRow_json["Data"].toString();

                ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,0,new QTableWidgetItem((QString)sysTime.data() ) );
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem((QString)ucTimeStamp.data() ));
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem((QString)FrameCounter.data() ));
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem((QString)SyncId.data() ));
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,new QTableWidgetItem((QString)DataString.data() ) );
            }

        }


    }

    NvM_DataLoadedFromExternalSourceFlag = true;
    on_GeneraReplotAllPlots_pb_clicked();
}





void MainWindow::on_LoadProject_pb_clicked()
{
    QString filter = "LfProject (*.lfp) ;; All files (*)";
    QString desktopPath = QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::DesktopLocation));
    QString file_name = QFileDialog::getOpenFileName(this,"choose file to overwrite",desktopPath,filter);
    LoadDataLineFollowerProjecrOrJson(file_name);
}

void MainWindow::on_actionAbout_triggered()
{
    QString MessageBoxString = QString("This is Line Follower Service application Release 1.1 \n"
                                       "\nAuthor: Teodor Rosolowski \n"
                                       "\nAppSite: github.com/trteodor/LineFollower_Light \n"
                                       "\nCreated using QT Framework 6.5 and QT Creator 10.0.2"
                                       "\n"
                                       "\n Feel free to use the App in your project!\n"
                                       "\nIf you liked this application please let me star in a git repo\n"
                                       "\n"
                                       "\nRed plots are associated with left side\n"
                                       ""
                                       "");

    QMessageBox::about(this,"About Line Follower Service App", MessageBoxString);
}

