#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QUrl>

/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QThread::currentThread()->setObjectName("Main Window Thread");



    ui->setupUi(this);

    this->setWindowTitle("LineFollower QT ServApp");


    /*All declared plots/ graph must be initialized!!!*/
    PlotMap.LfGraphInitialize(ui->MapViewWidget,QCPGraph::lsNone);
    PlotYawRate.LfGraphInitialize(ui->PlotYawRateW,QCPGraph::lsLine);
    PlotSpd.LfGraphInitialize(ui->PlotSpdW,QCPGraph::lsLine);
    PlotPosErr.LfGraphInitialize(ui->PlotPosErrW,QCPGraph::lsLine);
    PlotPidRegVal.LfGraphInitialize(ui->PlotPidRegValW,QCPGraph::lsLine);


    /*Initialize all needed connections for Bluetooth Data Manager*/
    BLE_InitializeQTConnections();


    connect(&NvM_ErrWeigthUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_ErrWeigthUpdateDelayTimerTimeout()));
    connect(&NvM_PidDatahUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_PidDatahUpdateDelayTimerTimeout()));
    connect(&NvM_VehCfghUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_VehCfgDataUpdateDelayTimerTimeout()));
    connect(&NvM_MotorsFactorsUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_MotorsFactorsDataUpdateDelayTimerTimeout()));
    connect(&NvM_EncoderCfgUpdateDelayTimer, SIGNAL(timeout()), this, SLOT(NvM_EncodersConfigDataUpdateDelayTimerTimeout()));


    addJoyStick(ui->RobotMoverXYGridLayout);

    /*Debug Table configure*/
    ui->DebugDataTable->setRowCount(1);
    ui->DebugDataTable->setColumnWidth(0,10);
    ui->DebugDataTable->setColumnWidth(1,10);
    ui->DebugDataTable->setColumnWidth(2,10);
    ui->DebugDataTable->setColumnWidth(3,10);
    ui->DebugDataTable->setColumnWidth(4,600);
    ui->DebugDataTable->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);


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

    ui->TextPwmToSpAFacL->setValidator(&dblValidator);
    ui->TextPwmToSpAFacR->setValidator(&dblValidator);
    ui->TextPwmToSpBFacL->setValidator(&dblValidator);
    ui->TextPwmToSpBFacR->setValidator(&dblValidator);


    QPalette pal = ui->BLE_RobotStart_Button->palette();
    pal.setColor(QPalette::Button, QColor(Qt::blue));
    ui->BLE_RobotStart_Button->setAutoFillBackground(true);
    ui->BLE_RobotStart_Button->setPalette(pal);
    ui->BLE_RobotStart_Button->update();


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


    QSettings settings("LfServiceApp", "BleDeviceName");
//    settings.setValue("CurrDeviceName", "Franek");
    QVariant CurrDevName = settings.value("CurrDeviceName");
    QString NewLineEditText = CurrDevName.toString();
//    qDebug() << NewLineEditText ;

    ui->BLE_AutoConnDevNameL->setText(NewLineEditText);


}
/*********************************************************************************************************/
MainWindow::~MainWindow()
{
    on_BLE_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    on_BLE_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    on_BLE_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    /*Send the command 3x to be sure that fakeProducer will be stopped, if not then re-connection may be impossible
     * - HW reset may be required
    */

    emit BLE_DisconnectDevice();

    QSettings settings("LfServiceApp", "BleDeviceName");
    QString NewSearchedBleDeviceName = ui->BLE_AutoConnDevNameL->text();
    settings.setValue("CurrDeviceName", NewSearchedBleDeviceName);

    delete ui;

    qDebug("Reached end");
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
    qDebug() << x << ", " << y;
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

void MainWindow::MainWin_UpdateNvM_VehCfgData(float ExpectedAvSpd, uint32_t BlinkLedSt, uint32_t TryDetEndLin)
{
    NvM_ExpectedAvSpeed = ExpectedAvSpd;
    NvM_BlinkLedSt = BlinkLedSt;
    NvM_TryDetEndLinSt = TryDetEndLin;



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
        SIGNAL(BleDatMngrSignal_PlotSpdAppendData(uint32_t,float,float) )
        ,this
        ,SLOT(MainWinPlot_PlotSpdAppendData(uint32_t,float,float) ) );

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


    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_UpdateErrorWeigthData(float,float,float,float,float,float,float,float,float,float,float,float))
        ,this
        ,SLOT(MainWin_UpdateNvmErrorWeigthData(float,float,float,float,float,float,float,float,float,float,float,float) ) );


    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_UpdateVehCfgData(float,uint32_t,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateNvM_VehCfgData(float, uint32_t, uint32_t) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_UpdatePidData(float,float,float,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateNvM_PidData(float,float,float,uint32_t) ) );


    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_UpdateMotorsFactors(uint32_t,uint32_t,uint32_t,uint32_t) )
        ,this
        ,SLOT(MainWin_UpdateMotorsFactors(uint32_t, uint32_t, uint32_t,uint32_t) ) );

    connect(
        &BleInputDataProcessingWrapper,
        SIGNAL(BleDatMngrSignal_UpdateEncoderCfgData(float,float) )
        ,this
        ,SLOT(MainWin_UpdateEncoderCfgData(float,float) ) );




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
                ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,new QTableWidgetItem(QString("DevNameMatched:Connecting ") ));
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

        ui->BLE_StatusLabel->setText("State:ConnReadyAcqData");
        QVariant variant= QColor (220,255,220);
        QString colcode = variant.toString();
        ui->BLE_StatusLabel->setAutoFillBackground(true);
        ui->BLE_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");


        ReadNvMDataFromLineFollower();

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
    int Index = ui->tabWidget_4->currentIndex();
    if(Index== 2)
    {
        PlotPosErr.LfGraph_UpdateReplot();
    }

    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.PosErrPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}

void MainWindow::MainWinPlot_PlotPidRegValReplot()
{
    int Index = ui->tabWidget_4->currentIndex();
    if(Index== 3)
    {
        PlotPidRegVal.LfGraph_UpdateReplot();
    }

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
    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.MapPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
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
    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.YawRatePlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
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


    BleInputDataProcessingWrapper.PlottingInfoMutex.lock();
    BleInputDataProcessingWrapper.SpdPlotPlottingState = FALSE;
    BleInputDataProcessingWrapper.PlottingInfoMutex.unlock();
}
/*********************************************************************************************************/


void MainWindow::on_BLE_SimulatorStartButton_clicked()
{
    char command[20];
    command[0] = (char)BleDataManager::BLE_SimulatorStart;

    for(int i=1; i<18; i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}
/*********************************************************************************************************/
void MainWindow::on_BLE_SimulatorSuspendButton_clicked()
{
    char command[20];
    command[0] = (char)BleDataManager::BLE_SimuAndTrueDataLoggingStop;

    for(int i=1; i<18; i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}

/*********************************************************************************************************/
void MainWindow::on_BLE_TrueLogStartButton_clicked()
{
    char command[20];
    command[0] = (char)BleDataManager::BLE_TrueBaseLoggingStart;

    for(int i=1; i<18; i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}
/*********************************************************************************************************/
void MainWindow::BLE_connectDevice()
{



    QString NewAutoConnDevName = ui->BLE_DetectedDeviceComboBox->itemText(ui->BLE_DetectedDeviceComboBox->currentIndex());
    ui->BLE_AutoConnDevNameL->setText(NewAutoConnDevName);

    emit BLE_connectToDevice(ui->BLE_DetectedDeviceComboBox->currentIndex());
}
/*********************************************************************************************************/
void MainWindow::on_BLE_DisconnectButton_clicked()
{
    on_BLE_SimulatorSuspendButton_clicked();
    QThread::msleep(30);
    on_BLE_SimulatorSuspendButton_clicked();
    QThread::msleep(30);
    on_BLE_SimulatorSuspendButton_clicked();
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

    ui->TextPwmToSpAFacL->clear();
    ui->TextPwmToSpAFacR->clear();
    ui->TextPwmToSpBFacL->clear();
    ui->TextPwmToSpBFacR->clear();

    ui->TextWheelBase->clear();
    ui->TextOneImpDist->clear();

    char command[20];
    command[0] = (char)BleDataManager::BLE_NvM_ErrWeigthSensorDataReq;
    for(int i=1; i<18; i++)
    {
        command[i] = 'B';
    }
    QByteArray Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    command[0] = (char)BleDataManager::BLE_NvM_LinePidRegDataReq;
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    command[0] = (char)BleDataManager::BLE_NvM_VehCfgReq;
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    command[0] = (char)BleDataManager::BLE_NvM_MotorsFactorsReq;
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    command[0] = (char)BleDataManager::BLE_NvM_EncoderModCfgReq;
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
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


    char command[20] = {0};
    QByteArray Helper;

    /***********************************************************************************/
    command[0] = (char)BleDataManager::BLE_NvM_ErrWeigthSensorData_part1;
    command[1] = SyncID;
    std::memcpy(&command[2],  &Err1ValueFloat, sizeof(float));
    std::memcpy(&command[6],  &Err2ValueFloat, sizeof(float));
    std::memcpy(&command[10], &Err3ValueFloat, sizeof(float));
    std::memcpy(&command[14], &Err4ValueFloat, sizeof(float));

    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    /***********************************************************************************/
    command[0] = (char)BleDataManager::BLE_NvM_ErrWeigthSensorData_part2;
    command[1] = SyncID;
    std::memcpy(&command[2],  &Err5ValueFloat, sizeof(float));
    std::memcpy(&command[6],  &Err6ValueFloat, sizeof(float));
    std::memcpy(&command[10], &Err7ValueFloat, sizeof(float));
    std::memcpy(&command[14], &Err8ValueFloat, sizeof(float));

    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    /***********************************************************************************/
    command[0] = (char)BleDataManager::BLE_NvM_ErrWeigthSensorData_part3;
    command[1] = SyncID;
    std::memcpy(&command[2],  &Err9ValueFloat, sizeof(float));
    std::memcpy(&command[6],  &Err10ValueFloat, sizeof(float));
    std::memcpy(&command[10], &Err11ValueFloat, sizeof(float));
    std::memcpy(&command[14], &ErrMValueFloat, sizeof(float));
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

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


    command[0] = (char)BleDataManager::BLE_NvM_LinePidRegData;
    command[1] = SyncID;
    std::memcpy(&command[2],  &PID_KPfloat, sizeof(float));
    std::memcpy(&command[6],  &PID_KIfloat, sizeof(float));
    std::memcpy(&command[10], &PID_KDfloat, sizeof(float));
    std::memcpy(&command[14], &ProbeTimeInt, sizeof(float));
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);


    /*********************************************************************************************************/
    /*********************************************************************************************************/
    /*********************************************************************************************************/

    QString ExpectedAvSpdText = ui->ExpectedAvSpdText->text();
    float ExpectedAvSpdFloat = ExpectedAvSpdText.toFloat();

    uint32_t BlinkingLedsState = (uint32_t) ui->BlinkingLedsStateCheckBox->isChecked();
    uint32_t TryDetEndLineMark = (uint32_t) ui->TryDetEndLineMarkCheckBox->isChecked();

////    qDebug() << "BlinkingLedsState" << BlinkingLedsState;
////    qDebug() << "TryDetEndLineMark" << TryDetEndLineMark;


    command[0] = (char)BleDataManager::BLE_NvM_VehCfgData;
    command[1] = SyncID;
    std::memcpy(&command[2],  &ExpectedAvSpdFloat, sizeof(float));
    std::memcpy(&command[6],  &BlinkingLedsState, sizeof(uint32_t));
    std::memcpy(&command[10], &TryDetEndLineMark, sizeof(uint32_t));
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);


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


    command[0] = (char)BleDataManager::BLE_NvM_MotorsFactorsData;
    command[1] = SyncID;
    std::memcpy(&command[2],  &FacA_LftU32, sizeof(uint32_t));
    std::memcpy(&command[6],  &FacA_RhtU32, sizeof(uint32_t));
    std::memcpy(&command[10], &FacB_LftU32, sizeof(uint32_t));
    std::memcpy(&command[14], &FacB_RhtU32, sizeof(uint32_t));
    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

    /*********************************************************************************************************/
    /*********************************************************************************************************/
    /*********************************************************************************************************/

    QString OneImpDistText = ui->TextOneImpDist->text();
    float OneImpDistF32 = OneImpDistText.toFloat();
    QString WheelBaseText = ui->TextWheelBase->text();
    float WheelBaseF32 = WheelBaseText.toFloat();


    command[0] = (char)BleDataManager::BLE_NvM_EncoderModCfgData;
    command[1] = SyncID;
    std::memcpy(&command[2],  &OneImpDistF32, sizeof(float));
    std::memcpy(&command[6],  &WheelBaseF32, sizeof(float));

//    qDebug() << "OneImpDistF32" << OneImpDistF32 << "WheelBaseF32" << WheelBaseF32;

    Helper = QByteArray::fromRawData(command,18);
    Helper.append("\n\r");
    BleInputDataProcessingWrapper.bleConnection.writeData(Helper);


    /*********************************************************************************************************/
    /*********************************************************************************************************/
    /*********************************************************************************************************/


    SyncID++;
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

void MainWindow::NvM_VehCfgDataUpdateDelayTimerTimeout()
{
    ui->ExpectedAvSpdText->setText(QString::number(NvM_ExpectedAvSpeed,'f',2) );

//    qDebug() << "NvM_BlinkLedSt" << NvM_BlinkLedSt;
//    qDebug() << "NvM_TryDetEndLinSt" << NvM_TryDetEndLinSt;

    (NvM_TryDetEndLinSt == 0)     ? ui->TryDetEndLineMarkCheckBox->setChecked(false) : ui->TryDetEndLineMarkCheckBox->setChecked(true);
    (NvM_BlinkLedSt == 0) ? ui->BlinkingLedsStateCheckBox->setChecked(false) : ui->BlinkingLedsStateCheckBox->setChecked(true);
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

void MainWindow::on_ClearLoggerButton_clicked()
{
        ui->DebugDataTable->clear();
        ui->DebugDataTable->setRowCount(0);
}



void MainWindow::on_BLE_RobotStop_Button_clicked()
{
        ui->BLE_TrueLogStartButton->setDisabled(false);
        ui->BLE_SimulatorSuspendButton->setDisabled(false);
        ui->BLE_SimulatorStartButton->setDisabled(false);

        ui->BLE_SimulatorSuspendButton->setDisabled(false);

        ui->BLE_TrueLogStartButton->setDisabled(false);

        ui->BLE_RobotStart_Button->setDisabled(false);
//        ui->ReadNvM_Button->setDisabled(false);
        ui->UpdateNvM_Button->setDisabled(false);
        ui->BLE_DisconnectButton->setDisabled(false);

        char command[20] = {0};
        QByteArray Helper;

        command[0] = (char)BleDataManager::BLE_RobotStop;
        command[1] = 0; //SyncID
        for(int i=1; i<18; i++)
        {
        command[i] = 'B';
        }
        Helper = QByteArray::fromRawData(command,18);
        Helper.append("\n\r");
        BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}


void MainWindow::on_BLE_RobotStart_Button_clicked()
{
        on_BLE_SimulatorSuspendButton_clicked();
        on_BLE_TrueLogStartButton_clicked();


        ui->BLE_TrueLogStartButton->setDisabled(true);
        ui->BLE_SimulatorSuspendButton->setDisabled(true);
        ui->BLE_SimulatorStartButton->setDisabled(true);
        ui->BLE_SimulatorSuspendButton->setDisabled(true);
        ui->BLE_TrueLogStartButton->setDisabled(true);
        ui->BLE_RobotStart_Button->setDisabled(true);
//        ui->ReadNvM_Button->setDisabled(true);
        ui->UpdateNvM_Button->setDisabled(true);
        ui->BLE_DisconnectButton->setDisabled(true);

        char command[20] = {0};
        QByteArray Helper;

        command[0] = (char)BleDataManager::BLE_RobotStart;
        command[1] = 0; //SyncID
        for(int i=1; i<18; i++)
        {
        command[i] = 'B';
        }
        Helper = QByteArray::fromRawData(command,18);
        Helper.append("\n\r");
        BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}


void MainWindow::on_UpdateExpectedAvSpd_clicked()
{
        QString ExpectedAvSpdText = ui->ExpectedAvSpdText->text();
        float ExpectedAvSpdFloat = ExpectedAvSpdText.toFloat();

        uint32_t BlinkingLedsState = (uint32_t) ui->BlinkingLedsStateCheckBox->isChecked();
        uint32_t TryDetEndLineMark = (uint32_t) ui->TryDetEndLineMarkCheckBox->isChecked();


        char command[20] = {0};
        QByteArray Helper;

        command[0] = (char)BleDataManager::BLE_NvM_VehCfgData;
        command[1] = 0; //SyncID
        std::memcpy(&command[2],  &ExpectedAvSpdFloat, sizeof(float));
        std::memcpy(&command[6],  &BlinkingLedsState, sizeof(uint32_t));
        std::memcpy(&command[10], &TryDetEndLineMark, sizeof(uint32_t));
        Helper = QByteArray::fromRawData(command,18);
        Helper.append("\n\r");
        BleInputDataProcessingWrapper.bleConnection.writeData(Helper);

}




void MainWindow::on_UpdateLfNameButton_clicked()
{
        char command[20] = {0};
        QByteArray Helper;
        command[0] = (char)BleDataManager::BLE_SetNewRobotName;
        command[1] = 0; //SyncID
        QString NewLineFollowerName = ui->NewLfNameText->text();
        QByteArray Helper2 = NewLineFollowerName.toUtf8();
        Helper = QByteArray::fromRawData(command,2);
        Helper = Helper + Helper2;
        qsizetype SizeNameFrame = Helper.size();

        while(SizeNameFrame < 20)
        {
        /*Just to be sure i'll send exactly 20bytes*/
        Helper.append('\0');
        SizeNameFrame = Helper.size();
        }

//        qDebug() << Helper;
        BleInputDataProcessingWrapper.bleConnection.writeData(Helper);
}


void MainWindow::on_GeneraReplotAllPlots_pb_clicked()
{
    PlotPosErr.LfGraph_UpdateReplot();
    PlotPidRegVal.LfGraph_UpdateReplot();
    PlotYawRate.LfGraph_UpdateReplot();
    PlotSpd.LfGraph_UpdateReplot();
    PlotMap.LfGraph_UpdateReplot();
}

