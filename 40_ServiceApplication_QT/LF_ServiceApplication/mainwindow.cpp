#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);



    /* Signal and Slots */
    /* Search Button */
    connect(ui->B_Search, SIGNAL(clicked()),&bleConnection, SLOT(startScan()));
    /* Connect Button */
    connect(ui->B_Connect,SIGNAL(clicked()), this, SLOT(connectDevice()));
//    /* Send Data Button */
    connect(ui->B_Send,SIGNAL(clicked()),this, SLOT(sendData()));
//    /* Bleutooth States */
    connect(&bleConnection, SIGNAL(changedState(bluetoothleUART::bluetoothleState)),this,SLOT(changedState(bluetoothleUART::bluetoothleState)));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::changedState(bluetoothleUART::bluetoothleState state){

    qDebug() << state;

    switch(state){

    case bluetoothleUART::Scanning:
    {
        ui->DetectedDeviceSelector->clear();

        ui->B_Connect->setEnabled(false);
        ui->B_Search->setEnabled(false);
        ui->DetectedDeviceSelector->setEnabled(false);

        ui->statusbar->showMessage("Searching for low energy devices...",1000);
        break;
    }


    case bluetoothleUART::ScanFinished:
    {
        bleConnection.getDeviceList(FoundDevices);

        if(!FoundDevices.empty()){

            for (int i = 0; i < FoundDevices.size(); i++)
            {
                ui->DetectedDeviceSelector->addItem(FoundDevices.at(i));
                qDebug() << ui->DetectedDeviceSelector->itemText(i);
            }

            /* Initialise Slot startConnect(int) -> button press ui->B_Connect */
            connect(this, SIGNAL(connectToDevice(int)),&bleConnection,SLOT(startConnect(int)));

            ui->B_Connect->setEnabled(true);
            ui->B_Search->setEnabled(true);
            ui->DetectedDeviceSelector->setEnabled(true);

            ui->statusbar->showMessage("Please select BLE device",1000);
        }
        else
        {
            ui->statusbar->showMessage("No Low Energy devices found",1000);
            ui->B_Search->setEnabled(true);
        }

        break;
    }

    case bluetoothleUART::Connecting:
    {
        ui->B_Connect->setEnabled(false);
        ui->B_Search->setEnabled(false);
        ui->DetectedDeviceSelector->setEnabled(false);

        ui->statusbar->showMessage("Connecting to device...",1000);
        break;
    }
    case bluetoothleUART::Connected:
    {
        ui->statusbar->showMessage("Device connected. Looking for service...",1000);
        break;
    }
    case bluetoothleUART::ServiceFound:
    {
        ui->statusbar->showMessage("Service found",1000);
        break;
    }
    case bluetoothleUART::AcquireData:
    {
        ui->B_Send->setEnabled(true);
        ui->lineSendDataEdit->setEnabled(true);

        /* Initialise Slot DataHandler(QString) - gets new data */
        connect(&bleConnection, SIGNAL(newData(QString)), this, SLOT(DataHandler(QString)));
        ui->statusbar->showMessage("Aquire data",1000);
        break;
    }
    default:
        //nothing for now
        break;


    }


}

void MainWindow::DataHandler(const QString &s){

    qDebug() << "ReceivedData:" << s;
}


void MainWindow::sendData(){

    bleConnection.writeData((QString)ui->lineSendDataEdit->text());
}

void MainWindow::connectDevice(){

    emit connectToDevice(ui->DetectedDeviceSelector->currentIndex());
}


