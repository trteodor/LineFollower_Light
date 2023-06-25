#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "bluetoothleuart.h"
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


    typedef enum
    {
        BLE_None = 0,
        BLE_ConfirmationTag,
        BLE_CommunicationStatistics,
        BLE_BaseDataReport_part1, /*Divide most necessary data into 1 compressed buffor to effienctly communicate*/
        BLE_BaseDataReport_part2,
        BLE_BaseDataReport_part3,
        BLE_SuspendFakeProducer,
        BLE_StartFakeProducer,
    }BLE_MessageID_t;


    /*
 * Modules should report the newest data then BLE module will transmit it
 * */
    typedef struct
    {
        float WhLftSp;
        float WhRhtSp;
        float YawRate;
        float PosX;
        float PosY;
        float TravelledDistance;
    }BLE_MapDataReport_t; /*Current size 6*4 = 24*/

    typedef struct
    {
        uint8_t SensorData[12];
        float PosError;
        uint8_t LastLeftLinePosConfidence;
        uint8_t LastRightLinePosConfidence;
    }BLE_SensorDataReport_t;  /*Current size= 12+4+1+1 = 18*/


    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        BLE_MapDataReport_t CurrMapData;
        BLE_SensorDataReport_t CurrSensorData;
    }BLE_LfDataReport_t; /*47bytes total size + 3*2 = 6*/


    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        uint16_t RingBufferRemainingSize;
        uint16_t RingBufferOverFlowCounter;
        uint16_t TransmisstedMessagesCounter;
        uint16_t RetransmissionCounter;
    }BLE_StatisticData_t;



private slots:
    void BLE_changedState(bluetoothleUART::bluetoothleState state);
    void BLE_DataHandler(const QByteArray &value);
    void BLE_connectDevice();


    void MapGraph_titleDoubleClick(QMouseEvent *event);
    void MapGraph_axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
    void MapGraph_legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
    void MapGraph_selectionChanged_selectionChanged();
    void MapGraph_mousePress();
    void MapGraph_mouseWheel();
    void MapGraph_addRandomGraph();
    void MapGraph_removeSelectedGraph();
    void MapGraph_removeAllGraphs();
    void MapGraph_contextMenuRequest(QPoint pos);
    void MapGraph_moveLegend();
    void MapGraph_Clicked(QCPAbstractPlottable *plottable, int dataIndex);

    void MapGraph_Initialize(void);
    void MapGraph_AppendData(uint32_t X_Pos,uint32_t Y_Pos);



    void BLE_InitializeQTConnections(void);
    void on_BLE_DisconnectButton_clicked();
    void on_BLE_BlockSignalsCheckBox_stateChanged(int arg1);
    void on_BLE_SuspendFakeProdButton_clicked();
    void on_BLE_ActivFakeProdButton_clicked();

    void on_GeneralPlotDataClear_pb_clicked();

signals:
    void BLE_connectToDevice(int i);
    void BLE_DisconnectDevice();
    void BLE_BlockData(bool Flag);

private:
    Ui::MainWindow *ui;
    bluetoothleUART bleConnection;
    QList<QString> FoundDevices;


    QPointer<QCPGraph> mGraph1;
    QPointer<QCPGraph> mGraph2;

    QVector<double> qv_x, qv_y;

    BLE_LfDataReport_t FullBaseData;

    void BLE_LfAppBaseData(const QByteArray &value,BLE_MessageID_t BLE_MessID);
    void UpdateDebugTableWithNewLfAppBaseData(void);
    void RefreshErrorIndicatorView(void);
    void BLE_CommunicationStatistics_Handler(const QByteArray &value);


};
#endif // MAINWINDOW_H
