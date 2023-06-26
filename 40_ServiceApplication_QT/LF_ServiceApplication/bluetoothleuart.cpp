#include "bluetoothleuart.h"

bluetoothleUART::bluetoothleUART() :
    m_currentDevice(QBluetoothDeviceInfo()),
    m_control(0),
    m_service(0),
    m_state(bluetoothleState::Idle)
{

    /* 1 Step: Bluetooth LE Device Discovery */
    m_deviceDiscoveryAgent = new QBluetoothDeviceDiscoveryAgent(this);


    /* Device Discovery Initialization */
    connect(m_deviceDiscoveryAgent, SIGNAL(deviceDiscovered(const QBluetoothDeviceInfo&)),
            this, SLOT(addDevice(const QBluetoothDeviceInfo&)));
    connect(m_deviceDiscoveryAgent, SIGNAL(error(QBluetoothDeviceDiscoveryAgent::Error)),
            this, SLOT(deviceScanError(QBluetoothDeviceDiscoveryAgent::Error)));
    connect(m_deviceDiscoveryAgent, SIGNAL(finished()), this, SLOT(scanFinished()));


}

bluetoothleUART::~bluetoothleUART(){


}



void bluetoothleUART::getDeviceList(QList<QString> &qlDevices){

        qlDevices = m_qlFoundDevices;
}



void bluetoothleUART::addDevice(const QBluetoothDeviceInfo &device)
{
    /* Is it a LE Bluetooth device? */
    if (device.coreConfigurations() & QBluetoothDeviceInfo::LowEnergyCoreConfiguration) {
        qWarning() << "Discovered LE Device name: " << device.name() << " Address: "
                   << device.address().toString();
        DeviceInfo *dev = new DeviceInfo(device);
        m_qlDevices.append(dev);
//                dev->getName()
        m_qlFoundDevices.append(((DeviceInfo*) m_qlDevices.at(m_qlDevices.size() -1  ))->getName());
        qDebug() << "Low Energy device found. Scanning for more...\n\r";

        qDebug() << "emit\n\r";
        emit changedState(NewDeviceDiscovered);
    }

}


void bluetoothleUART::scanFinished()
{
    qDebug() << "scanFinished\n\r";
}


void bluetoothleUART::deviceScanError(QBluetoothDeviceDiscoveryAgent::Error error)
{
    if (error == QBluetoothDeviceDiscoveryAgent::PoweredOffError)
        qDebug() << "The Bluetooth adaptor is powered off, power it on before doing discovery.";
    else if (error == QBluetoothDeviceDiscoveryAgent::InputOutputError)
        qDebug() << "Writing or reading from the device resulted in an error.";
    else
        qDebug() << "An unknown error has occurred.";
}



void bluetoothleUART::startScan(){

    qDeleteAll(m_qlDevices);
    m_qlDevices.clear();

    m_qlFoundDevices.clear();


    m_deviceDiscoveryAgent->start();
    setState(Scanning);

    qDebug()<< "Searching for low energy devices..." ;

}


void bluetoothleUART::startConnect(int i){

    m_deviceDiscoveryAgent->stop();
    qDebug() << "Discovery agent stopped";

    m_currentDevice.setDevice(((DeviceInfo*)m_qlDevices.at(i))->getDevice());

    qDebug() << &m_currentDevice;

    if (m_control) {
        m_control->disconnectFromDevice();
        delete m_control;
        m_control = nullptr;
    }

    if (!m_control)
    {
        /* 2 Step: QLowEnergyController */
        m_control = QLowEnergyController::createCentral(m_currentDevice.getDevice(), this);


        //    QLowEnergyController

        m_control ->setRemoteAddressType(QLowEnergyController::RandomAddress);

        connect(m_control, SIGNAL(serviceDiscovered(QBluetoothUuid)),
                this, SLOT(serviceDiscovered(QBluetoothUuid)));
        connect(m_control, SIGNAL(discoveryFinished()),
                this, SLOT(serviceScanDone()));
        connect(m_control, SIGNAL(error(QLowEnergyController::Error)),
                this, SLOT(controllerError(QLowEnergyController::Error)));
        connect(m_control, SIGNAL(connected()),
                this, SLOT(deviceConnected()));
        connect(m_control, SIGNAL(disconnected()),
                this, SLOT(deviceDisconnected()));

    }


    m_control->setRemoteAddressType(QLowEnergyController::PublicAddress);

    /* Start connecting to device */
    m_control->connectToDevice();
    setState(Connecting);


    m_previousAddress = m_currentDevice.getAddress();
}

void bluetoothleUART::DisconnectDevice()
{
    qDebug() << "DisconnectDevice";
    //disable notifications
    if (m_notificationDescTx.isValid() && m_service
        && m_notificationDescTx.value() == QByteArray::fromHex("0100")) {
        m_service->writeDescriptor(m_notificationDescTx, QByteArray::fromHex("0000"));
    } else {
        if (m_control)
            m_control->disconnectFromDevice();

        if(m_service)
        {
            delete m_service;
            m_service = nullptr;
        }
    }

    setState(Disconnected);
    setState(Idle);
}

void bluetoothleUART::BlockData(bool Flag)
{
    if(true == Flag && m_service){
        m_service->blockSignals(true);
    }
    else if(false == Flag && m_service){
        m_service->blockSignals(false);
    }
}

void bluetoothleUART::serviceDiscovered(const QBluetoothUuid &gatt){

    if(gatt==QBluetoothUuid(QUuid(UARTSERVICEUUID))){
        m_bFoundUARTService =true;
        qDebug() << "UART service found!";
    }
}



void bluetoothleUART::serviceScanDone(){


    delete m_service;
    m_service=0;

    if(m_bFoundUARTService){
       qDebug() << "Connecting to UART service...";
        m_service = m_control->createServiceObject(QBluetoothUuid(QUuid(UARTSERVICEUUID)),this);
    }

    if(!m_service){
        qDebug() <<"UART service not found";
        return;
    }

    /* 3 Step: Service Discovery */
    connect(m_service, SIGNAL(stateChanged(QLowEnergyService::ServiceState)),
            this, SLOT(serviceStateChanged(QLowEnergyService::ServiceState)));
    connect(m_service, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
            this, SLOT(updateData(QLowEnergyCharacteristic,QByteArray)));
    connect(m_service, SIGNAL(descriptorWritten(QLowEnergyDescriptor,QByteArray)),
            this, SLOT(confirmedDescriptorWrite(QLowEnergyDescriptor,QByteArray)));

    m_service->discoverDetails();
    setState(ServiceFound);

}

void bluetoothleUART::deviceDisconnected()
{

    qDebug() << "UART service disconnected";
    qWarning() << "Remote device disconnected";
    setState(Disconnected);
}

void bluetoothleUART::deviceConnected()
{
   qDebug() << "Device connected";
    m_control->discoverServices();
    setState(Connected);
}

void bluetoothleUART::controllerError(QLowEnergyController::Error error)
{
    qDebug() << "Cannot connect to remote device.";
    qWarning() << "Controller Error:" << error;
}


/* Slotes for QLowEnergyService */
void bluetoothleUART::serviceStateChanged(QLowEnergyService::ServiceState s)
{

    // A descriptoc can only be written if the service is in the ServiceDiscovered state
    switch (s) {
    case QLowEnergyService::RemoteServiceDiscovered:
    {

        //looking for the TX characteristic
        const QLowEnergyCharacteristic TxChar = m_service->characteristic(QBluetoothUuid(QUuid(TXUUID)));
        if (!TxChar.isValid()){
            qDebug() << "Tx characteristic not found";
            break;
        }

         //looking for the RX characteristic
        const QLowEnergyCharacteristic  RxChar = m_service->characteristic(QBluetoothUuid(QUuid(RXUUID)));
        if (!RxChar.isValid()) {
           qDebug() << "Rx characteristic not found";
            break;
        }


       // Bluetooth LE spec Where a characteristic can be notified, a Client Characteristic Configuration descriptor
       // shall be included in that characteristic as required by the Bluetooth Core Specification
       // Tx notify is enabled
        const QLowEnergyDescriptor m_notificationDescTx = TxChar.descriptor(
                     QBluetoothUuid::DescriptorType::ClientCharacteristicConfiguration);
        if (m_notificationDescTx.isValid()) {
            // enable notification
            m_service->writeDescriptor(m_notificationDescTx, QByteArray::fromHex("0100"));
            setState(AcquireData);

            }


        break;
    }
    default:
        //nothing for now
        break;
    }
}

void bluetoothleUART::updateData(const QLowEnergyCharacteristic &c,const QByteArray &value)
{
    // ignore any other characteristic change
    if (c.uuid() != QBluetoothUuid(QUuid(TXUUID)))
           return;



//    uint32_t ucTime = ((uint8_t)value.at(5) ) | ( (uint8_t)value.at(4) << 8) | ( (uint8_t)value.at(3) << 16);
//    qDebug("NewData: %x  %x   %x",value.at(5),value.at(4),value.at(3) );

//    qDebug("NewData: %d", ucTime);

//    qDebug("-------------------\n\r");
    emit newData(value);

}

void bluetoothleUART::confirmedDescriptorWrite(const QLowEnergyDescriptor &d,
                                         const QByteArray &value)
{
    if (d.isValid() && d == m_notificationDescTx && value == QByteArray("0000")) {
        //disabled notifications -> assume disconnect intent
        m_control->disconnectFromDevice();
        delete m_service;
        m_service = 0;
    }
}


void bluetoothleUART::writeData(QByteArray value){
    if(m_control && m_service)
    {
        const QLowEnergyCharacteristic  RxChar = m_service->characteristic(QBluetoothUuid(QUuid(RXUUID)));
        //    qDebug()<< s;
        QByteArray Data = value;
        //    Data.append("HelloWorld\n\r");
        m_service->writeCharacteristic(RxChar, Data,QLowEnergyService::WriteWithoutResponse);
    }

}



void bluetoothleUART::setState(bluetoothleUART::bluetoothleState newState)
{
    if (m_state == newState)
            return;

    m_state = newState;
    emit changedState(newState);
}

bluetoothleUART::bluetoothleState bluetoothleUART::getState() const {
    return m_state;
}
