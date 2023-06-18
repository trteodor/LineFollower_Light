/****************************************************************************
** Meta object code from reading C++ file 'bluetoothleuart.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../LF_ServiceApplication/bluetoothleuart.h"
#include <QtCore/qmetatype.h>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif


#include <memory>

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'bluetoothleuart.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS = QtMocHelpers::stringData(
    "bluetoothleUART",
    "newData",
    "",
    "s",
    "changedState",
    "bluetoothleUART::bluetoothleState",
    "newState",
    "addDevice",
    "QBluetoothDeviceInfo",
    "scanFinished",
    "deviceScanError",
    "QBluetoothDeviceDiscoveryAgent::Error",
    "serviceDiscovered",
    "QBluetoothUuid",
    "serviceScanDone",
    "controllerError",
    "QLowEnergyController::Error",
    "deviceConnected",
    "deviceDisconnected",
    "serviceStateChanged",
    "QLowEnergyService::ServiceState",
    "updateData",
    "QLowEnergyCharacteristic",
    "c",
    "value",
    "confirmedDescriptorWrite",
    "QLowEnergyDescriptor",
    "d",
    "startScan",
    "startConnect",
    "i",
    "bluetoothleState",
    "Idle",
    "Scanning",
    "ScanFinished",
    "Connecting",
    "Connected",
    "ServiceFound",
    "AcquireData"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS_t {
    uint offsetsAndSizes[78];
    char stringdata0[16];
    char stringdata1[8];
    char stringdata2[1];
    char stringdata3[2];
    char stringdata4[13];
    char stringdata5[34];
    char stringdata6[9];
    char stringdata7[10];
    char stringdata8[21];
    char stringdata9[13];
    char stringdata10[16];
    char stringdata11[38];
    char stringdata12[18];
    char stringdata13[15];
    char stringdata14[16];
    char stringdata15[16];
    char stringdata16[28];
    char stringdata17[16];
    char stringdata18[19];
    char stringdata19[20];
    char stringdata20[32];
    char stringdata21[11];
    char stringdata22[25];
    char stringdata23[2];
    char stringdata24[6];
    char stringdata25[25];
    char stringdata26[21];
    char stringdata27[2];
    char stringdata28[10];
    char stringdata29[13];
    char stringdata30[2];
    char stringdata31[17];
    char stringdata32[5];
    char stringdata33[9];
    char stringdata34[13];
    char stringdata35[11];
    char stringdata36[10];
    char stringdata37[13];
    char stringdata38[12];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS_t qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS = {
    {
        QT_MOC_LITERAL(0, 15),  // "bluetoothleUART"
        QT_MOC_LITERAL(16, 7),  // "newData"
        QT_MOC_LITERAL(24, 0),  // ""
        QT_MOC_LITERAL(25, 1),  // "s"
        QT_MOC_LITERAL(27, 12),  // "changedState"
        QT_MOC_LITERAL(40, 33),  // "bluetoothleUART::bluetoothleS..."
        QT_MOC_LITERAL(74, 8),  // "newState"
        QT_MOC_LITERAL(83, 9),  // "addDevice"
        QT_MOC_LITERAL(93, 20),  // "QBluetoothDeviceInfo"
        QT_MOC_LITERAL(114, 12),  // "scanFinished"
        QT_MOC_LITERAL(127, 15),  // "deviceScanError"
        QT_MOC_LITERAL(143, 37),  // "QBluetoothDeviceDiscoveryAgen..."
        QT_MOC_LITERAL(181, 17),  // "serviceDiscovered"
        QT_MOC_LITERAL(199, 14),  // "QBluetoothUuid"
        QT_MOC_LITERAL(214, 15),  // "serviceScanDone"
        QT_MOC_LITERAL(230, 15),  // "controllerError"
        QT_MOC_LITERAL(246, 27),  // "QLowEnergyController::Error"
        QT_MOC_LITERAL(274, 15),  // "deviceConnected"
        QT_MOC_LITERAL(290, 18),  // "deviceDisconnected"
        QT_MOC_LITERAL(309, 19),  // "serviceStateChanged"
        QT_MOC_LITERAL(329, 31),  // "QLowEnergyService::ServiceState"
        QT_MOC_LITERAL(361, 10),  // "updateData"
        QT_MOC_LITERAL(372, 24),  // "QLowEnergyCharacteristic"
        QT_MOC_LITERAL(397, 1),  // "c"
        QT_MOC_LITERAL(399, 5),  // "value"
        QT_MOC_LITERAL(405, 24),  // "confirmedDescriptorWrite"
        QT_MOC_LITERAL(430, 20),  // "QLowEnergyDescriptor"
        QT_MOC_LITERAL(451, 1),  // "d"
        QT_MOC_LITERAL(453, 9),  // "startScan"
        QT_MOC_LITERAL(463, 12),  // "startConnect"
        QT_MOC_LITERAL(476, 1),  // "i"
        QT_MOC_LITERAL(478, 16),  // "bluetoothleState"
        QT_MOC_LITERAL(495, 4),  // "Idle"
        QT_MOC_LITERAL(500, 8),  // "Scanning"
        QT_MOC_LITERAL(509, 12),  // "ScanFinished"
        QT_MOC_LITERAL(522, 10),  // "Connecting"
        QT_MOC_LITERAL(533, 9),  // "Connected"
        QT_MOC_LITERAL(543, 12),  // "ServiceFound"
        QT_MOC_LITERAL(556, 11)   // "AcquireData"
    },
    "bluetoothleUART",
    "newData",
    "",
    "s",
    "changedState",
    "bluetoothleUART::bluetoothleState",
    "newState",
    "addDevice",
    "QBluetoothDeviceInfo",
    "scanFinished",
    "deviceScanError",
    "QBluetoothDeviceDiscoveryAgent::Error",
    "serviceDiscovered",
    "QBluetoothUuid",
    "serviceScanDone",
    "controllerError",
    "QLowEnergyController::Error",
    "deviceConnected",
    "deviceDisconnected",
    "serviceStateChanged",
    "QLowEnergyService::ServiceState",
    "updateData",
    "QLowEnergyCharacteristic",
    "c",
    "value",
    "confirmedDescriptorWrite",
    "QLowEnergyDescriptor",
    "d",
    "startScan",
    "startConnect",
    "i",
    "bluetoothleState",
    "Idle",
    "Scanning",
    "ScanFinished",
    "Connecting",
    "Connected",
    "ServiceFound",
    "AcquireData"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSbluetoothleUARTENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       1,  143, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,  104,    2, 0x06,    1 /* Public */,
       4,    1,  107,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       7,    1,  110,    2, 0x08,    5 /* Private */,
       9,    0,  113,    2, 0x08,    7 /* Private */,
      10,    1,  114,    2, 0x08,    8 /* Private */,
      12,    1,  117,    2, 0x08,   10 /* Private */,
      14,    0,  120,    2, 0x08,   12 /* Private */,
      15,    1,  121,    2, 0x08,   13 /* Private */,
      17,    0,  124,    2, 0x08,   15 /* Private */,
      18,    0,  125,    2, 0x08,   16 /* Private */,
      19,    1,  126,    2, 0x08,   17 /* Private */,
      21,    2,  129,    2, 0x08,   19 /* Private */,
      25,    2,  134,    2, 0x08,   22 /* Private */,
      28,    0,  139,    2, 0x08,   25 /* Private */,
      29,    1,  140,    2, 0x08,   26 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, 0x80000000 | 5,    6,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 8,    2,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 11,    2,
    QMetaType::Void, 0x80000000 | 13,    2,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 16,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 20,    3,
    QMetaType::Void, 0x80000000 | 22, QMetaType::QByteArray,   23,   24,
    QMetaType::Void, 0x80000000 | 26, QMetaType::QByteArray,   27,   24,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   30,

 // enums: name, alias, flags, count, data
      31,   31, 0x0,    7,  148,

 // enum data: key, value
      32, uint(bluetoothleUART::Idle),
      33, uint(bluetoothleUART::Scanning),
      34, uint(bluetoothleUART::ScanFinished),
      35, uint(bluetoothleUART::Connecting),
      36, uint(bluetoothleUART::Connected),
      37, uint(bluetoothleUART::ServiceFound),
      38, uint(bluetoothleUART::AcquireData),

       0        // eod
};

Q_CONSTINIT const QMetaObject bluetoothleUART::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSbluetoothleUARTENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<bluetoothleUART, std::true_type>,
        // method 'newData'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QString, std::false_type>,
        // method 'changedState'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bluetoothleUART::bluetoothleState, std::false_type>,
        // method 'addDevice'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QBluetoothDeviceInfo &, std::false_type>,
        // method 'scanFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'deviceScanError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QBluetoothDeviceDiscoveryAgent::Error, std::false_type>,
        // method 'serviceDiscovered'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QBluetoothUuid &, std::false_type>,
        // method 'serviceScanDone'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'controllerError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QLowEnergyController::Error, std::false_type>,
        // method 'deviceConnected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'deviceDisconnected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'serviceStateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QLowEnergyService::ServiceState, std::false_type>,
        // method 'updateData'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QLowEnergyCharacteristic &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QByteArray &, std::false_type>,
        // method 'confirmedDescriptorWrite'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QLowEnergyDescriptor &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QByteArray &, std::false_type>,
        // method 'startScan'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'startConnect'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>
    >,
    nullptr
} };

void bluetoothleUART::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<bluetoothleUART *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->newData((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->changedState((*reinterpret_cast< std::add_pointer_t<bluetoothleUART::bluetoothleState>>(_a[1]))); break;
        case 2: _t->addDevice((*reinterpret_cast< std::add_pointer_t<QBluetoothDeviceInfo>>(_a[1]))); break;
        case 3: _t->scanFinished(); break;
        case 4: _t->deviceScanError((*reinterpret_cast< std::add_pointer_t<QBluetoothDeviceDiscoveryAgent::Error>>(_a[1]))); break;
        case 5: _t->serviceDiscovered((*reinterpret_cast< std::add_pointer_t<QBluetoothUuid>>(_a[1]))); break;
        case 6: _t->serviceScanDone(); break;
        case 7: _t->controllerError((*reinterpret_cast< std::add_pointer_t<QLowEnergyController::Error>>(_a[1]))); break;
        case 8: _t->deviceConnected(); break;
        case 9: _t->deviceDisconnected(); break;
        case 10: _t->serviceStateChanged((*reinterpret_cast< std::add_pointer_t<QLowEnergyService::ServiceState>>(_a[1]))); break;
        case 11: _t->updateData((*reinterpret_cast< std::add_pointer_t<QLowEnergyCharacteristic>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[2]))); break;
        case 12: _t->confirmedDescriptorWrite((*reinterpret_cast< std::add_pointer_t<QLowEnergyDescriptor>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[2]))); break;
        case 13: _t->startScan(); break;
        case 14: _t->startConnect((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QBluetoothDeviceInfo >(); break;
            }
            break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QBluetoothUuid >(); break;
            }
            break;
        case 7:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QLowEnergyController::Error >(); break;
            }
            break;
        case 10:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QLowEnergyService::ServiceState >(); break;
            }
            break;
        case 11:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QLowEnergyCharacteristic >(); break;
            }
            break;
        case 12:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QLowEnergyDescriptor >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (bluetoothleUART::*)(QString );
            if (_t _q_method = &bluetoothleUART::newData; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (bluetoothleUART::*)(bluetoothleUART::bluetoothleState );
            if (_t _q_method = &bluetoothleUART::changedState; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject *bluetoothleUART::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *bluetoothleUART::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSbluetoothleUARTENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int bluetoothleUART::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void bluetoothleUART::newData(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void bluetoothleUART::changedState(bluetoothleUART::bluetoothleState _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
