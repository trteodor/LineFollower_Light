#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "bluetoothleuart.h"

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
    void changedState(bluetoothleUART::bluetoothleState state);
    void DataHandler(const QString &s);
    void connectDevice();
    void sendData();

signals:
    void connectToDevice(int i);

private:
    Ui::MainWindow *ui;
    bluetoothleUART bleConnection;
    QList<QString> FoundDevices;
};
#endif // MAINWINDOW_H
