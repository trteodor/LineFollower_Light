/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QComboBox *DetectedDeviceSelector;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QPushButton *B_Send;
    QLineEdit *lineSendDataEdit;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *B_Search;
    QPushButton *B_Connect;
    QMenuBar *menubar;
    QStatusBar *statusbar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(595, 600);
        MainWindow->setDocumentMode(false);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        DetectedDeviceSelector = new QComboBox(centralwidget);
        DetectedDeviceSelector->setObjectName("DetectedDeviceSelector");
        DetectedDeviceSelector->setEnabled(true);
        DetectedDeviceSelector->setGeometry(QRect(110, 360, 341, 24));
        widget = new QWidget(centralwidget);
        widget->setObjectName("widget");
        widget->setGeometry(QRect(110, 400, 346, 61));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        B_Send = new QPushButton(widget);
        B_Send->setObjectName("B_Send");
        B_Send->setEnabled(false);

        horizontalLayout->addWidget(B_Send);

        lineSendDataEdit = new QLineEdit(widget);
        lineSendDataEdit->setObjectName("lineSendDataEdit");
        lineSendDataEdit->setEnabled(false);

        horizontalLayout->addWidget(lineSendDataEdit);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        B_Search = new QPushButton(widget);
        B_Search->setObjectName("B_Search");

        horizontalLayout_2->addWidget(B_Search);

        B_Connect = new QPushButton(widget);
        B_Connect->setObjectName("B_Connect");

        horizontalLayout_2->addWidget(B_Connect);


        verticalLayout->addLayout(horizontalLayout_2);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 595, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName("toolBar");
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        B_Send->setText(QCoreApplication::translate("MainWindow", "SendData", nullptr));
        B_Search->setText(QCoreApplication::translate("MainWindow", "SearchButton", nullptr));
        B_Connect->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
