#ifndef BLUETOOTHCLASSIC_H
#define BLUETOOTHCLASSIC_H

#include <QObject>

#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothSocket>

class bluetoothClassic : public QObject
{
public:
    bluetoothClassic();
    ~bluetoothClassic();

    void bluetoothSendDataToDevice(QByteArray Data);

signals:
    /*Signals for higher layer - for example GUI*/
    void bluetoothSignalDeviceDiscovered(QString name);
    void bluetoothSignalDiscoveryFinished(void);

    void bluetoothSignalConnectingStart();
    void bluetoothSignalConnectionEstablished(void);
    void bluetoothSignalConnectionInterrupted(void);

    void bluetoothSignalNewDataReceived(char *Data, uint32_t size);


private slots:
    /*User slots for example from GUI*/
    void bluetoothStartDiscoveryDevices(void);
    void bluetoothDisconnect(void);
    void bluetootConnectToDeviceByName(QString DevName);

    /*Private slots needed to co-opearte with library functions*/
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
