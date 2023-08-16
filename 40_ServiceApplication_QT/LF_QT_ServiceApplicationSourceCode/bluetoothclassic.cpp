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
    qDebug() << "bluetoothClassic:Constructor called";
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
//    qDebug() << "bluetoothClassic:bluetoothStartDiscoveryDevices:  called";
    bluetoothDiscoveryAgent->start();
}

void bluetoothClassic::bluetootConnectToDeviceByName(QString DevName)
{


    qDebug() << "bluetoothClassic:bluetootConnectToDeviceByName Name:" << DevName ;

    int indexOfDevice = -1;

    for (int i = 0; i < FoundDevicesListNames.size(); ++i) {
        if (this->FoundDevicesListNames.at(i) == DevName)
        {
            indexOfDevice = i;
//            qDebug() << "bluetootConnectToDeviceByName DevFound";
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
//    qDebug() << "bluetoothClassic:bluetoothDisconnect:  called";

    bluetoothClassicSocket->disconnectFromService();
}


void bluetoothClassic::bluetoothSearchingFinished(void)
{
//    qDebug() << "bluetoothClassic:bluetoothSearchingFinished:  called";

    emit bluetoothSignalDiscoveryFinished();
}

void bluetoothClassic::bluetoothDeviceDiscovered(const QBluetoothDeviceInfo &device)
{
//    qDebug() << "bluetoothClassic:bluetoothDeviceDiscovered:  called";

    this->FoundDevicesListNames.append( device.name());
    this->FoundDevicesListAddresses.append(device.address().toString() );
    emit bluetoothSignalDeviceDiscovered(device.name() );
}

void bluetoothClassic::bluetoothConnectionEstablished() {
//    qDebug() << "bluetoothClassic:bluetoothConnectionEstablished:  called";
    emit bluetoothSignalConnectionEstablished();
}

void bluetoothClassic::bluetoothConnectionInterrupted() {
//    qDebug() << "bluetoothClassic:bluetoothConnectionInterrupted:  called";
    emit bluetoothSignalConnectionInterrupted();
}

void  bluetoothClassic::bluetoothSendDataToDevice(QByteArray Data)
{

    uint32_t DataTransmitSize = Data.size();
    if(DataTransmitSize != BLU_SINGLE_TR_MESSAGE_SIZE)
    {
        qDebug() << "bluetoothSendDataToDevice unexpected Size:" << DataTransmitSize << "data:" << Data;
        //      qDebug() << "bluetoothSendDataToDevice Size:" << DataTransmitSize;
    }


  if(this->bluetoothClassicSocket->isOpen() && this->bluetoothClassicSocket->isWritable()) {
    this->bluetoothClassicSocket->write(Data);
  } else {
    qDebug() << "Can't transmit message bluetooth bluetoothClassicSocket isn't writable";
  }
}

void bluetoothClassic::bluetoothSocketReadyToRead(void)
{
  while(bluetoothClassicSocket->bytesAvailable() >99 ) {
        static char Data[5000] = {0};
        uint32_t Size = bluetoothClassicSocket->read(&Data[0],100);
        emit bluetoothSignalNewDataReceived(Data,100);

//        qDebug() << "SyncID:" << (uint8_t)Data[1] << "Size:" << Size <<"FullFramesCount:" << FullFramesCount;

//        static uint32_t CleanerAfterWakeUpFlag = 100;
//        if(CleanerAfterWakeUpFlag != 0 && bluetoothClassicSocket->bytesAvailable() > 0)
//        {
//            CleanerAfterWakeUpFlag = CleanerAfterWakeUpFlag - 1;
//            Size = bluetoothClassicSocket->read(&Data[0],5000);
//            qDebug() << "Err?: SyncID" << (uint8_t)Data[1] << "Size:" << Size <<"FullFramesCount:" << FullFramesCount;
//        }

    }
}


