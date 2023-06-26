QT       += core gui bluetooth

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    bluetoothleuart.cpp \
    deviceinfo.cpp \
    qcustomplot.cpp

HEADERS += \
    mainwindow.h \
    bluetoothleuart.h \
    deviceinfo.h \
    qcustomplot.h

FORMS += \
    mainwindow.ui

TRANSLATIONS += \
    LF_ServiceApplication_en_150.ts
CONFIG += lrelease
CONFIG += embed_translations

RESOURCES += qdarkstyle/dark/darkstyle.qrc
RESOURCES += qdarkstyle/light/lightstyle.qrc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
