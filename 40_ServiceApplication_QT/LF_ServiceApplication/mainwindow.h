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
        BLE_BaseSensorData, /*FRAME: Line Sensors 1-12 as 8bit, PosError, ConfidenceLine Position Left and Right*/
        BLE_ExtraSensorData,
        BLE_BaseMapData,
        BLE_ExtraMapData,

    }BLE_MessageID_t;


    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        float WhLftSp;
        float WhRhtSp;
        float YawRate;
        float PosX;
        float PosY;
        float TravelledDistance;
    }BLE_MapData_t;

    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        uint8_t SensorData[12];
        float PosError;
        uint8_t LastLeftLinePosConfidence;
        uint8_t LastRightLinePosConfidence;
    }BLE_SensorData_t;


private slots:
    void changedState(bluetoothleUART::bluetoothleState state);
    void DataHandler(const QByteArray &value);
    void connectDevice();
    void sendData();


    void titleDoubleClick(QMouseEvent *event);
    void axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
    void legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
    void selectionChanged();
    void mousePress();
    void mouseWheel();
    void addRandomGraph();
    void removeSelectedGraph();
    void removeAllGraphs();
    void contextMenuRequest(QPoint pos);
    void moveLegend();
    void graphClicked(QCPAbstractPlottable *plottable, int dataIndex);

    void InitializeMapGraph(void);
    void GraphAppendData(uint32_t X_Pos,uint32_t Y_Pos);

    void on_pushButton_3_clicked();

    void on_BLE_DisconnectButton_clicked();

    void on_BLE_BlockSignalsCheckBox_stateChanged(int arg1);

signals:
    void connectToDevice(int i);
    void DisconnectBLE_Dev();
    void BLE_BlockData(bool Flag);

private:
    Ui::MainWindow *ui;
    bluetoothleUART bleConnection;
    QList<QString> FoundDevices;


    QPointer<QCPGraph> mGraph1;
    QPointer<QCPGraph> mGraph2;

    QVector<double> qv_x, qv_y;




    void BLE_MapData_Handler(const QByteArray &value,BLE_MessageID_t BLE_MessID);
    void BLE_CommunicationStatistics_Handler(const QByteArray &value);


};
#endif // MAINWINDOW_H
