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
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableView>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionHello;
    QAction *actionAbout;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *pushButton_6;
    QPushButton *pushButton_10;
    QPushButton *pushButton_9;
    QPushButton *pushButton_8;
    QPushButton *pushButton_7;
    QPushButton *pushButton_5;
    QTableView *tableView_2;
    QTabWidget *tabWidget_2;
    QWidget *tab_5;
    QWidget *widget;
    QWidget *tab_6;
    QWidget *widget_2;
    QWidget *widget_3;
    QWidget *Reserved1;
    QWidget *tab_8;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuAbout;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QDockWidget *dockWidget_2;
    QWidget *dockWidgetContents_9;
    QGridLayout *gridLayout_2;
    QPushButton *B_Connect;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QHBoxLayout *horizontalLayout_8;
    QLineEdit *lineEdit_5;
    QLineEdit *lineEdit_6;
    QLineEdit *lineEdit_7;
    QLineEdit *lineEdit_8;
    QWidget *layoutWidget_2;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QHBoxLayout *horizontalLayout_10;
    QLineEdit *lineEdit_9;
    QLineEdit *lineEdit_10;
    QLineEdit *lineEdit_11;
    QLineEdit *lineEdit_12;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_11;
    QPushButton *pushButton_12;
    QPushButton *pushButton_13;
    QPushButton *pushButton_14;
    QWidget *widget2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_6;
    QLineEdit *lineEdit;
    QLineEdit *lineEdit_2;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_4;
    QWidget *tab_2;
    QWidget *tab_3;
    QWidget *tab_4;
    QPushButton *B_Send;
    QComboBox *DetectedDeviceSelector;
    QPushButton *B_Search;
    QLineEdit *lineSendDataEdit;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QDockWidget *dockWidget_3;
    QWidget *dockWidgetContents_10;
    QGridLayout *gridLayout_3;
    QTableView *tableView_3;
    QLabel *label_13;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(877, 624);
        MainWindow->setMinimumSize(QSize(800, 600));
        MainWindow->setSizeIncrement(QSize(1, 1));
        MainWindow->setDocumentMode(false);
        actionHello = new QAction(MainWindow);
        actionHello->setObjectName("actionHello");
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName("actionAbout");
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName("gridLayout");
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        pushButton_6 = new QPushButton(centralwidget);
        pushButton_6->setObjectName("pushButton_6");

        horizontalLayout_5->addWidget(pushButton_6);

        pushButton_10 = new QPushButton(centralwidget);
        pushButton_10->setObjectName("pushButton_10");

        horizontalLayout_5->addWidget(pushButton_10);

        pushButton_9 = new QPushButton(centralwidget);
        pushButton_9->setObjectName("pushButton_9");

        horizontalLayout_5->addWidget(pushButton_9);

        pushButton_8 = new QPushButton(centralwidget);
        pushButton_8->setObjectName("pushButton_8");

        horizontalLayout_5->addWidget(pushButton_8);

        pushButton_7 = new QPushButton(centralwidget);
        pushButton_7->setObjectName("pushButton_7");

        horizontalLayout_5->addWidget(pushButton_7);

        pushButton_5 = new QPushButton(centralwidget);
        pushButton_5->setObjectName("pushButton_5");

        horizontalLayout_5->addWidget(pushButton_5);


        gridLayout->addLayout(horizontalLayout_5, 3, 0, 1, 1);

        tableView_2 = new QTableView(centralwidget);
        tableView_2->setObjectName("tableView_2");
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(tableView_2->sizePolicy().hasHeightForWidth());
        tableView_2->setSizePolicy(sizePolicy);
        tableView_2->setMinimumSize(QSize(300, 30));
        tableView_2->setMaximumSize(QSize(16777215, 30));
        tableView_2->setSizeIncrement(QSize(1, 0));

        gridLayout->addWidget(tableView_2, 1, 0, 1, 1);

        tabWidget_2 = new QTabWidget(centralwidget);
        tabWidget_2->setObjectName("tabWidget_2");
        tab_5 = new QWidget();
        tab_5->setObjectName("tab_5");
        widget = new QWidget(tab_5);
        widget->setObjectName("widget");
        widget->setGeometry(QRect(0, 1, 661, 290));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy1);
        widget->setMinimumSize(QSize(600, 290));
        widget->setSizeIncrement(QSize(1, 1));
        tabWidget_2->addTab(tab_5, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName("tab_6");
        widget_2 = new QWidget(tab_6);
        widget_2->setObjectName("widget_2");
        widget_2->setGeometry(QRect(-1, -1, 661, 151));
        widget_3 = new QWidget(tab_6);
        widget_3->setObjectName("widget_3");
        widget_3->setGeometry(QRect(-1, 160, 661, 151));
        tabWidget_2->addTab(tab_6, QString());
        Reserved1 = new QWidget();
        Reserved1->setObjectName("Reserved1");
        tabWidget_2->addTab(Reserved1, QString());
        tab_8 = new QWidget();
        tab_8->setObjectName("tab_8");
        tabWidget_2->addTab(tab_8, QString());

        gridLayout->addWidget(tabWidget_2, 2, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 877, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName("menuFile");
        menuAbout = new QMenu(menubar);
        menuAbout->setObjectName("menuAbout");
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName("toolBar");
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        dockWidget_2 = new QDockWidget(MainWindow);
        dockWidget_2->setObjectName("dockWidget_2");
        dockWidget_2->setMinimumSize(QSize(320, 410));
        dockWidgetContents_9 = new QWidget();
        dockWidgetContents_9->setObjectName("dockWidgetContents_9");
        gridLayout_2 = new QGridLayout(dockWidgetContents_9);
        gridLayout_2->setObjectName("gridLayout_2");
        B_Connect = new QPushButton(dockWidgetContents_9);
        B_Connect->setObjectName("B_Connect");

        gridLayout_2->addWidget(B_Connect, 1, 2, 1, 1);

        tabWidget = new QTabWidget(dockWidgetContents_9);
        tabWidget->setObjectName("tabWidget");
        sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy);
        tabWidget->setMinimumSize(QSize(250, 130));
        tabWidget->setSizeIncrement(QSize(1, 1));
        tabWidget->setTabPosition(QTabWidget::South);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setIconSize(QSize(18, 18));
        tabWidget->setElideMode(Qt::ElideNone);
        tabWidget->setDocumentMode(false);
        tab = new QWidget();
        tab->setObjectName("tab");
        layoutWidget = new QWidget(tab);
        layoutWidget->setObjectName("layoutWidget");
        layoutWidget->setGeometry(QRect(10, 80, 261, 52));
        verticalLayout_3 = new QVBoxLayout(layoutWidget);
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName("label_5");

        horizontalLayout_7->addWidget(label_5);

        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName("label_6");

        horizontalLayout_7->addWidget(label_6);

        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName("label_7");

        horizontalLayout_7->addWidget(label_7);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName("label_8");

        horizontalLayout_7->addWidget(label_8);


        verticalLayout_3->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName("horizontalLayout_8");
        lineEdit_5 = new QLineEdit(layoutWidget);
        lineEdit_5->setObjectName("lineEdit_5");

        horizontalLayout_8->addWidget(lineEdit_5);

        lineEdit_6 = new QLineEdit(layoutWidget);
        lineEdit_6->setObjectName("lineEdit_6");

        horizontalLayout_8->addWidget(lineEdit_6);

        lineEdit_7 = new QLineEdit(layoutWidget);
        lineEdit_7->setObjectName("lineEdit_7");

        horizontalLayout_8->addWidget(lineEdit_7);

        lineEdit_8 = new QLineEdit(layoutWidget);
        lineEdit_8->setObjectName("lineEdit_8");

        horizontalLayout_8->addWidget(lineEdit_8);


        verticalLayout_3->addLayout(horizontalLayout_8);

        layoutWidget_2 = new QWidget(tab);
        layoutWidget_2->setObjectName("layoutWidget_2");
        layoutWidget_2->setGeometry(QRect(10, 130, 261, 52));
        verticalLayout_4 = new QVBoxLayout(layoutWidget_2);
        verticalLayout_4->setSpacing(0);
        verticalLayout_4->setObjectName("verticalLayout_4");
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName("horizontalLayout_9");
        label_9 = new QLabel(layoutWidget_2);
        label_9->setObjectName("label_9");

        horizontalLayout_9->addWidget(label_9);

        label_10 = new QLabel(layoutWidget_2);
        label_10->setObjectName("label_10");

        horizontalLayout_9->addWidget(label_10);

        label_11 = new QLabel(layoutWidget_2);
        label_11->setObjectName("label_11");

        horizontalLayout_9->addWidget(label_11);

        label_12 = new QLabel(layoutWidget_2);
        label_12->setObjectName("label_12");

        horizontalLayout_9->addWidget(label_12);


        verticalLayout_4->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName("horizontalLayout_10");
        lineEdit_9 = new QLineEdit(layoutWidget_2);
        lineEdit_9->setObjectName("lineEdit_9");

        horizontalLayout_10->addWidget(lineEdit_9);

        lineEdit_10 = new QLineEdit(layoutWidget_2);
        lineEdit_10->setObjectName("lineEdit_10");

        horizontalLayout_10->addWidget(lineEdit_10);

        lineEdit_11 = new QLineEdit(layoutWidget_2);
        lineEdit_11->setObjectName("lineEdit_11");

        horizontalLayout_10->addWidget(lineEdit_11);

        lineEdit_12 = new QLineEdit(layoutWidget_2);
        lineEdit_12->setObjectName("lineEdit_12");

        horizontalLayout_10->addWidget(lineEdit_12);


        verticalLayout_4->addLayout(horizontalLayout_10);

        widget1 = new QWidget(tab);
        widget1->setObjectName("widget1");
        widget1->setGeometry(QRect(10, 0, 261, 26));
        horizontalLayout = new QHBoxLayout(widget1);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_11 = new QPushButton(widget1);
        pushButton_11->setObjectName("pushButton_11");

        horizontalLayout->addWidget(pushButton_11);

        pushButton_12 = new QPushButton(widget1);
        pushButton_12->setObjectName("pushButton_12");

        horizontalLayout->addWidget(pushButton_12);

        pushButton_13 = new QPushButton(widget1);
        pushButton_13->setObjectName("pushButton_13");

        horizontalLayout->addWidget(pushButton_13);

        pushButton_14 = new QPushButton(widget1);
        pushButton_14->setObjectName("pushButton_14");

        horizontalLayout->addWidget(pushButton_14);

        widget2 = new QWidget(tab);
        widget2->setObjectName("widget2");
        widget2->setGeometry(QRect(10, 30, 261, 52));
        verticalLayout = new QVBoxLayout(widget2);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        label = new QLabel(widget2);
        label->setObjectName("label");

        horizontalLayout_2->addWidget(label);

        label_2 = new QLabel(widget2);
        label_2->setObjectName("label_2");

        horizontalLayout_2->addWidget(label_2);

        label_3 = new QLabel(widget2);
        label_3->setObjectName("label_3");

        horizontalLayout_2->addWidget(label_3);

        label_4 = new QLabel(widget2);
        label_4->setObjectName("label_4");

        horizontalLayout_2->addWidget(label_4);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        lineEdit = new QLineEdit(widget2);
        lineEdit->setObjectName("lineEdit");

        horizontalLayout_6->addWidget(lineEdit);

        lineEdit_2 = new QLineEdit(widget2);
        lineEdit_2->setObjectName("lineEdit_2");

        horizontalLayout_6->addWidget(lineEdit_2);

        lineEdit_3 = new QLineEdit(widget2);
        lineEdit_3->setObjectName("lineEdit_3");

        horizontalLayout_6->addWidget(lineEdit_3);

        lineEdit_4 = new QLineEdit(widget2);
        lineEdit_4->setObjectName("lineEdit_4");

        horizontalLayout_6->addWidget(lineEdit_4);


        verticalLayout->addLayout(horizontalLayout_6);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName("tab_2");
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName("tab_3");
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName("tab_4");
        tabWidget->addTab(tab_4, QString());

        gridLayout_2->addWidget(tabWidget, 3, 0, 1, 3);

        B_Send = new QPushButton(dockWidgetContents_9);
        B_Send->setObjectName("B_Send");
        B_Send->setEnabled(false);

        gridLayout_2->addWidget(B_Send, 2, 2, 1, 1);

        DetectedDeviceSelector = new QComboBox(dockWidgetContents_9);
        DetectedDeviceSelector->setObjectName("DetectedDeviceSelector");
        DetectedDeviceSelector->setEnabled(true);
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(DetectedDeviceSelector->sizePolicy().hasHeightForWidth());
        DetectedDeviceSelector->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(DetectedDeviceSelector, 0, 0, 1, 3);

        B_Search = new QPushButton(dockWidgetContents_9);
        B_Search->setObjectName("B_Search");

        gridLayout_2->addWidget(B_Search, 1, 0, 1, 2);

        lineSendDataEdit = new QLineEdit(dockWidgetContents_9);
        lineSendDataEdit->setObjectName("lineSendDataEdit");
        lineSendDataEdit->setEnabled(false);

        gridLayout_2->addWidget(lineSendDataEdit, 2, 0, 1, 2);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(3);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout_2->setSizeConstraint(QLayout::SetMinimumSize);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        pushButton_2 = new QPushButton(dockWidgetContents_9);
        pushButton_2->setObjectName("pushButton_2");

        horizontalLayout_3->addWidget(pushButton_2);

        pushButton = new QPushButton(dockWidgetContents_9);
        pushButton->setObjectName("pushButton");

        horizontalLayout_3->addWidget(pushButton);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        pushButton_3 = new QPushButton(dockWidgetContents_9);
        pushButton_3->setObjectName("pushButton_3");

        horizontalLayout_4->addWidget(pushButton_3);

        pushButton_4 = new QPushButton(dockWidgetContents_9);
        pushButton_4->setObjectName("pushButton_4");

        horizontalLayout_4->addWidget(pushButton_4);


        verticalLayout_2->addLayout(horizontalLayout_4);


        gridLayout_2->addLayout(verticalLayout_2, 4, 0, 1, 3);

        dockWidget_2->setWidget(dockWidgetContents_9);
        MainWindow->addDockWidget(Qt::LeftDockWidgetArea, dockWidget_2);
        dockWidget_3 = new QDockWidget(MainWindow);
        dockWidget_3->setObjectName("dockWidget_3");
        dockWidget_3->setMinimumSize(QSize(218, 100));
        dockWidget_3->setBaseSize(QSize(0, 0));
        dockWidget_3->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents_10 = new QWidget();
        dockWidgetContents_10->setObjectName("dockWidgetContents_10");
        gridLayout_3 = new QGridLayout(dockWidgetContents_10);
        gridLayout_3->setObjectName("gridLayout_3");
        tableView_3 = new QTableView(dockWidgetContents_10);
        tableView_3->setObjectName("tableView_3");
        sizePolicy.setHeightForWidth(tableView_3->sizePolicy().hasHeightForWidth());
        tableView_3->setSizePolicy(sizePolicy);
        tableView_3->setMinimumSize(QSize(200, 50));
        tableView_3->setSizeIncrement(QSize(1, 1));

        gridLayout_3->addWidget(tableView_3, 1, 0, 1, 1);

        label_13 = new QLabel(dockWidgetContents_10);
        label_13->setObjectName("label_13");

        gridLayout_3->addWidget(label_13, 0, 0, 1, 1);

        dockWidget_3->setWidget(dockWidgetContents_10);
        MainWindow->addDockWidget(Qt::BottomDockWidgetArea, dockWidget_3);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuAbout->menuAction());
        menuFile->addAction(actionHello);
        menuAbout->addAction(actionAbout);
        toolBar->addAction(actionHello);
        toolBar->addAction(actionAbout);

        retranslateUi(MainWindow);

        tabWidget_2->setCurrentIndex(0);
        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionHello->setText(QCoreApplication::translate("MainWindow", "Hello", nullptr));
        actionAbout->setText(QCoreApplication::translate("MainWindow", "About", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_10->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_9->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_8->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_7->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_5), QCoreApplication::translate("MainWindow", "MapData", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_6), QCoreApplication::translate("MainWindow", "YawRate_Spd", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(Reserved1), QCoreApplication::translate("MainWindow", "Reserved1", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_8), QCoreApplication::translate("MainWindow", "Reserved2", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuAbout->setTitle(QCoreApplication::translate("MainWindow", "About", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        B_Connect->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        pushButton_11->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_12->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_13->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_14->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "BaseCfg", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "SensorCfg", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "AdvCfg", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "SpecialCfg", nullptr));
        B_Send->setText(QCoreApplication::translate("MainWindow", "SendData", nullptr));
        B_Search->setText(QCoreApplication::translate("MainWindow", "Scan BLE Devices", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "Logs:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
