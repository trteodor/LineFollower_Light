#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtConcurrent>
#include "qcustomplot.h"

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
//    void BLE_DataHandler(const QByteArray &value);
    void BLE_connectDevice();

//    void BLE_LfAppBaseData(const QByteArray &value,BleDataManager::BLE_MessageID_t BLE_MessID);
    void BLE_UpdateDebugTableWithNewLfAppBaseData(void);

    void BLE_InitializeQTConnections(void);
    void on_BLE_DisconnectButton_clicked();
    void on_BLE_BlockSignalsCheckBox_stateChanged(int arg1);
    void on_BLE_SuspendFakeProdButton_clicked();
    void on_BLE_ActivFakeProdButton_clicked();
    void on_GeneralPlotDataClear_pb_clicked();


    void MainWinPlot_MapUpdate(void);
    void MainWinPlot_YawRateUpdate(void);
    void MainWinPlot_SpdUpdate(void);

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


};
#endif // MAINWINDOW_H
