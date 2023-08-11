#ifndef BLUETOOTHCLASSIC_H
#define BLUETOOTHCLASSIC_H


#include <QObject>

#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothSocket>

#define BLU_SINGLE_MESSAGE_SIZE 100
#define BLU_SINGLE_TR_MESSAGE_SIZE 100

class bluetoothClassic : public QObject
{
    Q_OBJECT

public:
    bluetoothClassic();
    ~bluetoothClassic();

    void bluetoothSendDataToDevice(QByteArray Data);

signals:
    /*Connect Signals for higher layer - for example GUI*/
    void bluetoothSignalDeviceDiscovered(QString name);
    void bluetoothSignalDiscoveryFinished(void);

    void bluetoothSignalConnectingStart();
    void bluetoothSignalConnectionEstablished(void);
    void bluetoothSignalConnectionInterrupted(void);

    /*Received data signals event for data manager*/
    void bluetoothSignalNewDataReceived(char *Data, uint32_t size);


public slots:
    /*User public slots - slots for example for GUI*/
    void bluetoothStartDiscoveryDevices(void);
    void bluetoothDisconnect(void);
    void bluetootConnectToDeviceByName(QString DevName);

private slots:
    /*Private slots needed to co-opearte with QT library functions*/
    void bluetoothDeviceDiscovered(const QBluetoothDeviceInfo &device);
    void bluetoothSearchingFinished(void);

    void bluetoothSocketReadyToRead();
    void bluetoothConnectionEstablished();
    void bluetoothConnectionInterrupted();
    
private:
    QList<QString> FoundDevicesListNames;
    QList<QString> FoundDevicesListAddresses;

    QBluetoothSocket *bluetoothClassicSocket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol);

    QBluetoothDeviceDiscoveryAgent *bluetoothDiscoveryAgent = new QBluetoothDeviceDiscoveryAgent;
    QString string;

};

#endif // BLUETOOTHCLASSIC_H
