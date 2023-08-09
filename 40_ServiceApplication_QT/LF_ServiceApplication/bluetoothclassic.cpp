#include "bluetoothclassic.h"

bluetoothClassic::bluetoothClassic()
{
    connect(bluetoothDiscoveryAgent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)),
            this,SLOT(bluetoothDeviceDiscovered(QBluetoothDeviceInfo)));

    connect(this->bluetoothDiscoveryAgent, SIGNAL(finished()),this, SLOT(bluetoothSearchingFinished()));


    bluetoothClassicSocket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol);

    connect(bluetoothClassicSocket, SIGNAL(connected()),this, SLOT(bluetoothConnectionEstablished()));
    connect(bluetoothClassicSocket, SIGNAL(disconnected()),this, SLOT(bluetoothConnectionInterrupted()));
    connect(bluetoothClassicSocket, SIGNAL(readyRead()),this, SLOT(bluetoothSocketReadyToRead()));
}
bluetoothClassic::~bluetoothClassic()
{
    bluetoothClassicSocket->disconnectFromService();
    qDebug() << "Delete and DisconnectBluetoothClassic";
    delete bluetoothDiscoveryAgent;
    delete bluetoothClassicSocket;
}

void bluetoothClassic::bluetoothStartDiscoveryDevices(void)
{
    bluetoothDiscoveryAgent->start();
}

void bluetoothClassic::bluetootConnectToDeviceByName(QString *DevName)
{
    int indexOfDevice = -1;

    for (int i = 0; i < this->FoundDevicesListNames.size(); ++i) {
        if (this->FoundDevicesListNames.at(i) == *DevName)
        {
            indexOfDevice = i;
            qDebug() << "bluetootConnectToDeviceByName DevFound";
            break;
        }
    }

    if(indexOfDevice != (-1) )
    {
        static const QString serviceUuid(QStringLiteral("00001101-0000-1000-8000-00805F9B34FB"));
        bluetoothClassicSocket->connectToService(QBluetoothAddress(
                                    this->FoundDevicesListAddresses.at(indexOfDevice)),
                                    QBluetoothUuid(serviceUuid), QIODevice::ReadWrite);
        emit bluetoothSignalConnectingStart();
    }
}

void bluetoothClassic::bluetoothDisconnect(void)
{
    bluetoothClassicSocket->disconnectFromService();
}


void bluetoothClassic::bluetoothSearchingFinished(void)
{
    emit bluetoothSignalDiscoveryFinished();
}

void bluetoothClassic::bluetoothDeviceDiscovered(const QBluetoothDeviceInfo &device)
{
    QList<QString> FoundDevicesListNames;
    QList<QString> FoundDevicesListAddresses;

    this->FoundDevicesListNames.append( device.name());
    this->FoundDevicesListAddresses.append(device.address().toString() );
    emit bluetoothSignalDeviceDiscovered(device.name() );
}

void bluetoothClassic::bluetoothConnectionEstablished() {
    qDebug() << "bluetoothConnectionEstablished";
    emit bluetoothSignalConnectionEstablished();
}

void bluetoothClassic::bluetoothConnectionInterrupted() {
    qDebug() << "bluetoothConnectionInterrupted";
    emit bluetoothSignalConnectionInterrupted();
}

void  bluetoothClassic::bluetoothSendDataToDevice(QByteArray Data)
{
  if(this->bluetoothClassicSocket->isOpen() && this->bluetoothClassicSocket->isWritable()) {
    this->bluetoothClassicSocket->write(Data);
  } else {
    qDebug() << "Can't transmit message bluetooth bluetoothClassicSocket isn't writable";
  }
}

void bluetoothClassic::bluetoothSocketReadyToRead(void)
{
//    while(bluetoothClassicSocket->bytesAvailable() ) {
//        char Data[255];
//        uint32_t Size = bluetoothClassicSocket->read(Data,255);
//        qDebug() <<"BlutoothMessage:  " << Data;
//    }
    while(this->bluetoothClassicSocket->canReadLine()) {
        char Data[255] = {0};
        uint32_t Size = bluetoothClassicSocket->read(Data,255);

        emit bluetoothSignalNewDataReceived(Data,Size);

        qDebug() <<"BlutoothMessage:  " << Data;
        //    QString line = this->bluetoothClassicSocket->readLine();
        //    qDebug() << line;
        
    }
}


