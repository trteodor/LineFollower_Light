#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtConcurrent>
#include "BleDataManager.h"
#include "GenericLfQCP.h"

//#include <QInputDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();



    enum JoyType {
        XY,
        HorizontalOnly,
        VerticalOnly
    };

    void addJoyStick(QLayout *layout_, JoyType type = XY);



private slots:

    void joystick_moved(double x, double y);

    void BLE_changedState(bluetoothleUART::bluetoothleState state);
    void BLE_connectDevice();


    void BLE_InitializeQTConnections(void);

    void on_BLE_DisconnectButton_clicked();
    void on_BLE_SimulatorStartButton_clicked();
    void on_BLE_SimulatorSuspendButton_clicked();
    void on_BLE_TrueLogStartButton_clicked();
    void on_BLE_RobotStop_Button_clicked();
    void on_BLE_RobotStart_Button_clicked();


    void on_UpdateExpectedAvSpd_clicked();
    void on_UpdateLfNameButton_clicked();

    void on_GeneralPlotDataClear_pb_clicked();


    void MainWin_UpdateNvmErrorWeigthData( float ErrW1,float ErrW2,float ErrW3,float ErrW4,float ErrW5,float ErrW6,
                                        float ErrW7,float ErrW8,float ErrW9,float ErrW10,float ErrW11,float ErrWM);


    void MainWin_UpdateNvM_PidData(float Kp, float Ki, float Kd, uint32_t ProbeTime);
    void MainWin_UpdateNvM_VehCfgData(float ExpectedAvSpd,uint32_t BlinkLedSt, uint32_t TryDetEndLin);

    void MainWin_RefreshErrorIndicatorView( uint8_t S0,uint8_t S1,uint8_t S2,uint8_t S3,uint8_t S4,uint8_t S5,
                                           uint8_t S6,uint8_t S7,uint8_t S8,uint8_t S9,uint8_t S10,uint8_t S11,
                                           uint8_t RightLinePosConfif,uint8_t LeftLinePosConfif,
                                           float PosError);

    void MainWin_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor = QColor( 255,255,255) );

    void MainWin_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
                                                           uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter);



    void MainWin_DebugTable_ScrollToBottom();

    void MainWinPlot_PlotMapReplot(void);
    void MainWinPlot_PlotYawRateReplot(void);
    void MainWinPlot_PlotSpdReplot(void);
    void MainWinPlot_PlotPosErrReplot(void);
    void MainWinPlot_PlotPidRegValReplot(void);

    void MainWinPlot_PlotMapAppendData(float PosX, float PosY);
    void MainWinPlot_PlotYawRateAppendData(uint32_t FrameId, float YrValue);
    void MainWinPlot_PlotSpdAppendData(uint32_t FrameId, float SpdValue);
    void MainWinPlot_PlotPosErrAppendData(uint32_t FrameId, float PossErrValue);
    void MainWinPlot_PlotPidRegValAppendData(uint32_t FrameId, float PidRegVal);

    void on_EnableBaseDataLogging_clicked(bool checked);
    void on_DebugTable_DisableBaseDataLogging_clicked(bool checked);

    void on_ReadNvM_Button_clicked();

    void on_UpdateNvM_Button_clicked();



    void on_ClearLoggerButton_clicked();


    void NvM_ErrWeigthUpdateDelayTimerTimeout();
    void NvM_PidDatahUpdateDelayTimerTimeout();
    void NvM_VehCfgDataUpdateDelayTimerTimeout();


    void on_GeneraReplotAllPlots_pb_clicked();

signals:
    void BLE_connectToDevice(int i);
    void BLE_DisconnectDevice();
    void BLE_BlockData(bool Flag);

private:
    Ui::MainWindow *ui;

    QTimer NvM_ErrWeigthUpdateDelayTimer;
    QTimer NvM_PidDatahUpdateDelayTimer;
    QTimer NvM_VehCfghUpdateDelayTimer;

    float  NVM_ErrWeitghtsTabHolder[12];

    float  NvM_PID_Kp;
    float  NvM_PID_Ki;
    float  NvM_PID_Kd;
    uint32_t  NvM_ProbeTim;

    float  NvM_ExpectedAvSpeed;
    uint32_t  NvM_BlinkLedSt;
    uint32_t  NvM_TryDetEndLinSt;

    QList<QString> FoundDevices;
    BleDataManager BleInputDataProcessingWrapper;


    QDoubleValidator dblValidator;

    void ReadNvMDataFromLineFollower();
    void RefreshErrorIndicatorView(void);
    void BLE_CommunicationStatistics_Handler(const QByteArray &value);

    GenericLfQCP PlotMap;
    GenericLfQCP PlotYawRate;
    GenericLfQCP PlotSpd;
    GenericLfQCP PlotPosErr;
    GenericLfQCP PlotPidRegVal;

};
#endif // MAINWINDOW_H
