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






private slots:
    void BLE_changedState(bluetoothleUART::bluetoothleState state);
    void BLE_connectDevice();


    void BLE_InitializeQTConnections(void);

    void on_BLE_DisconnectButton_clicked();

    void on_BLE_SuspendFakeProdButton_clicked();
    void on_BLE_ActivFakeProdButton_clicked();
    void on_GeneralPlotDataClear_pb_clicked();

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

signals:
    void BLE_connectToDevice(int i);
    void BLE_DisconnectDevice();
    void BLE_BlockData(bool Flag);

private:
    Ui::MainWindow *ui;


    QList<QString> FoundDevices;
    BleDataManager BleInputDataProcessingWrapper;

    void RefreshErrorIndicatorView(void);
    void BLE_CommunicationStatistics_Handler(const QByteArray &value);

    GenericLfQCP PlotMap;
    GenericLfQCP PlotYawRate;
    GenericLfQCP PlotSpd;
    GenericLfQCP PlotPosErr;
    GenericLfQCP PlotPidRegVal;

};
#endif // MAINWINDOW_H
