#include "mainwindow.h"
#include "ui_mainwindow.h"




/*****************************************/
/*Tools functions start*/
/**/

uint32_t ConvToUint32(const QByteArray &value, uint8_t ShiftValue)
{
    return (((uint8_t)value.at(ShiftValue) ) | ( (uint8_t)value.at(ShiftValue+1) << 8) \
            | ( (uint8_t)value.at(ShiftValue+2) << 16) | ( (uint8_t)value.at(ShiftValue+3) << 24));

}


float ieee_uint32_AsBitsTo_float32(uint32_t f)
{
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}

/*Tools functions end */
/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    connect(this, SIGNAL(BLE_BlockData(bool)),&bleConnection,SLOT(BlockData(bool)));
    /* Signal and Slots */
    /* Search Button */
    connect(ui->BLE_ScanButton, SIGNAL(clicked()),&bleConnection, SLOT(startScan()));

    /* Connect Button */
    connect(ui->BLE_ConnectButton,SIGNAL(clicked()), this, SLOT(connectDevice()));
    connect(this, SIGNAL(DisconnectBLE_Dev() ),&bleConnection,SLOT(DisconnectDevice() ));
//    /* Send Data Button */
//    connect(ui->B_Send,SIGNAL(clicked()),this, SLOT(sendData()));
//    /* Bleutooth States */
    connect(&bleConnection, SIGNAL(changedState(bluetoothleUART::bluetoothleState)),this,SLOT(changedState(bluetoothleUART::bluetoothleState)));


//    ui->MapViewWidget
    ui->MapViewWidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                    QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->MapViewWidget->xAxis->setRange(-8, 8);
    ui->MapViewWidget->yAxis->setRange(-5, 5);
    ui->MapViewWidget->axisRect()->setupFullAxesBox();

//    ui->MapViewWidget->plotLayout()->insertRow(0);
//    QCPTextElement *title = new QCPTextElement(ui->MapViewWidget, "Interaction Example", QFont("sans", 17, QFont::Bold));
//    ui->MapViewWidget->plotLayout()->addElement(0, 0, title);

    ui->MapViewWidget->xAxis->setLabel("x Axis");
    ui->MapViewWidget->yAxis->setLabel("y Axis");
    ui->MapViewWidget->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    ui->MapViewWidget->legend->setFont(legendFont);
    ui->MapViewWidget->legend->setSelectedFont(legendFont);
    ui->MapViewWidget->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items

    InitializeMapGraph();
    ui->MapViewWidget->rescaleAxes();

    // connect slot that ties some axis selections together (especially opposite axes):
    connect(ui->MapViewWidget, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
    // connect slots that takes care that when an axis is selected, only that direction can be dragged and zoomed:
    connect(ui->MapViewWidget, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress()));
    connect(ui->MapViewWidget, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel()));

    // make bottom and left axes transfer their ranges to top and right axes:
    connect(ui->MapViewWidget->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->MapViewWidget->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->MapViewWidget->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->MapViewWidget->yAxis2, SLOT(setRange(QCPRange)));

    // connect some interaction slots:
    connect(ui->MapViewWidget, SIGNAL(axisDoubleClick(QCPAxis*,QCPAxis::SelectablePart,QMouseEvent*)), this, SLOT(axisLabelDoubleClick(QCPAxis*,QCPAxis::SelectablePart)));
    connect(ui->MapViewWidget, SIGNAL(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*,QMouseEvent*)), this, SLOT(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*)));
//    connect(title, SIGNAL(doubleClicked(QMouseEvent*)), this, SLOT(titleDoubleClick(QMouseEvent*)));

    // connect slot that shows a message in the status bar when a graph is clicked:
    connect(ui->MapViewWidget, SIGNAL(plottableClick(QCPAbstractPlottable*,int,QMouseEvent*)), this, SLOT(graphClicked(QCPAbstractPlottable*,int)));

    // setup policy and connect slot for context menu popup:
    ui->MapViewWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->MapViewWidget, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));


    ui->tableWidge_2->setRowCount(1);
    ui->tableWidge_2->setColumnWidth(0,50);
    ui->tableWidge_2->setColumnWidth(1,50);
    ui->tableWidge_2->setColumnWidth(2,28);
    ui->tableWidge_2->setColumnWidth(3,10);
    ui->tableWidge_2->setColumnWidth(4,400);

    ui->tableWidge_2->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
}
/*********************************************************************************************************/
MainWindow::~MainWindow()
{
    delete ui;
}
/*********************************************************************************************************/
void MainWindow::changedState(bluetoothleUART::bluetoothleState state){

    static uint8_t deviceCounter_l = 0;

    qDebug() << state;

    switch(state){

    case bluetoothleUART::Scanning:
    {
        ui->BLE_StatusLabel->setText("State:Scanning");
        ui->BLE_DetectedDeviceComboBox->clear();

        ui->BLE_ConnectButton->setEnabled(false);
        ui->BLE_ScanButton->setEnabled(false);
        ui->BLE_DetectedDeviceComboBox->setEnabled(true);

        ui->statusbar->showMessage("Searching for low energy devices...",1000);
        break;
    }

    case bluetoothleUART::NewDeviceDiscovered:
    {
        ui->BLE_StatusLabel->setText("State:NewDevDet");
        bleConnection.getDeviceList(FoundDevices);

        if(!FoundDevices.empty()){

            ui->BLE_DetectedDeviceComboBox->addItem(FoundDevices.at(deviceCounter_l));
            /* Initialise Slot startConnect(int) -> button press ui->BLE_ConnectButton */
            connect(this, SIGNAL(connectToDevice(int)),&bleConnection,SLOT(startConnect(int)));

            ui->BLE_ConnectButton->setEnabled(true);
            ui->BLE_ScanButton->setEnabled(true);
            ui->BLE_DetectedDeviceComboBox->setEnabled(true);

            QString AutoConnDevName = ui->BLE_AutoConnDevNameL->text();

            if( ui->BLE_DetectedDeviceComboBox->itemText(deviceCounter_l) ==  AutoConnDevName
                && ui->BLE_AutoConnCheckBox->isChecked() )
            {
                ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
                ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,new QTableWidgetItem(QString("Expected dev. name Matched: Autoconnection ") ));
                emit connectToDevice(deviceCounter_l);
            }
            else
            {
                ui->statusbar->showMessage("Please select BLE device",1000);
            }
            deviceCounter_l++;
        }

        break;
    }

    case bluetoothleUART::ScanFinished:
    {
        ui->BLE_StatusLabel->setText("State:ScanFinished");
    }
    case bluetoothleUART::Connecting:
    {
                ui->BLE_StatusLabel->setText("State:Connecting");
        ui->BLE_ConnectButton->setEnabled(false);
        ui->BLE_ScanButton->setEnabled(false);
//        ui->BLE_DetectedDeviceComboBox->setEnabled(false);

        ui->statusbar->showMessage("Connecting to device...",1000);
        break;
    }
    case bluetoothleUART::Connected:
    {
        ui->BLE_StatusLabel->setText("State:Connected");
        break;
    }
    case bluetoothleUART::Disconnected:
    {
        ui->BLE_StatusLabel->setText("State:Disconnected");
        ui->BLE_ConnectButton->setEnabled(true);
        ui->BLE_ScanButton->setEnabled(true);
        deviceCounter_l = 0;
        break;
    }
    case bluetoothleUART::ServiceFound:
    {
        break;
    }
    case bluetoothleUART::AcquireData:
    {
        connect(&bleConnection, SIGNAL(newData(QByteArray)), this, SLOT(DataHandler(QByteArray)));
        ui->statusbar->showMessage("Aquire data",1000);
        break;
    }
    default:
        break;


    }


}

void MainWindow::BLE_CommunicationStatistics_Handler(const QByteArray &value)
{
//    static uint8_t SyncIDCommStatistics;
}




void MainWindow::BLE_MapData_Handler(const QByteArray &value,BLE_MessageID_t BLE_MessID)
{
    static volatile uint8_t DupaTable[50];
    static uint32_t PrevSyncId = 0U;
    uint8_t _inputSyncId = ((uint8_t)value.at(1)) ;



    static uint16_t FrameCounter = 0U;
    static volatile bool ExpectingExtraData = false;
    static MainWindow::BLE_MapData_t IncomingMapData;
    static MainWindow::BLE_MapData_t FullMapData;

    if( (false == ExpectingExtraData) && (BLE_MessID == BLE_MessageID_t::BLE_BaseMapData)  && (PrevSyncId != _inputSyncId))
    {
        IncomingMapData.ucTimeStamp = ConvToUint32(value,2);
        IncomingMapData.WhLftSp = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6)) ;
        IncomingMapData.WhRhtSp =ieee_uint32_AsBitsTo_float32(ConvToUint32(value,10)) ;
        IncomingMapData.SyncId = _inputSyncId;
        ExpectingExtraData = true;
    }
    else
    {
        if( (true == ExpectingExtraData) && (PrevSyncId == _inputSyncId) && (BLE_MessID == BLE_MessageID_t::BLE_ExtraMapData) )
        {
                IncomingMapData.PosX = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,2)) ;
                IncomingMapData.PosY = ieee_uint32_AsBitsTo_float32(ConvToUint32(value,6)) ;

                /*Copy full input MapData*/
                FullMapData = IncomingMapData;
                ExpectingExtraData = false;


                QString RgtWhlSpdSimu_s = QString::number(FullMapData.WhLftSp,'f',3);
                QString LftWhlSpdSimu_s = QString::number(FullMapData.WhRhtSp,'f',3);
                QString MyData = QString("pX: %1 |pY: %2 |L_sp: %3 |R_Sp: %4")
                                     .arg(FullMapData.PosX).arg(FullMapData.PosY)
                                     .arg(LftWhlSpdSimu_s)
                                     .arg(RgtWhlSpdSimu_s) ;

                ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
                ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,1,new QTableWidgetItem(QString::number(FullMapData.ucTimeStamp) ));
                ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
                ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,3,new QTableWidgetItem(QString::number(FullMapData.SyncId)));
                ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,
                                          new QTableWidgetItem((QString)MyData.data() ) );

                GraphAppendData(FullMapData.PosX,FullMapData.PosY);
        }
        else
        {
                if(PrevSyncId != _inputSyncId)  /*Frame Already handled ignore*/
                {
                    /*Ignore synchronization error detected*/
                    QString ExpectingExtraData_s = QString::number(ExpectingExtraData);
                    QString SyncErroMessage = QString("MapDatSyncErr cID:'%1'|pID:'%2'|mID:'%3' eeData:'%4' ")
                                                  .arg(_inputSyncId).arg(PrevSyncId).arg(BLE_MessID),arg(ExpectingExtraData_s);
                    ui->tableWidge_2->insertRow(ui->tableWidge_2->rowCount() );
                    ui->tableWidge_2->setItem(ui->tableWidge_2->rowCount() -1 ,4,new QTableWidgetItem(SyncErroMessage));
                    ui->tableWidge_2->item(ui->tableWidge_2->rowCount() -1 , 4) -> setData(Qt::BackgroundRole, QColor (250,0,0));
                    ExpectingExtraData = false;
                }

        }
    }


    PrevSyncId = _inputSyncId;
}



/*********************************************************************************************************/
void MainWindow::DataHandler(const QByteArray &value){

    static volatile BLE_MessageID_t BLE_MessageID;
    BLE_MessageID = ((BLE_MessageID_t)value.at(0) );

    qDebug() <<  "NewFrameID:" << BLE_MessageID << "SyncID:" << ((uint8_t)value.at(1) );
    QString message = QString("NewFrID '%1'").arg(BLE_MessageID);
    ui->statusbar->showMessage(message, 500);

    switch(BLE_MessageID)
    {
        case BLE_MessageID_t::BLE_CommunicationStatistics:
        {
            BLE_CommunicationStatistics_Handler(value);
            break;
        }

        case BLE_MessageID_t::BLE_BaseMapData:
        {
            BLE_MapData_Handler(value,BLE_MessageID);
            break;
        }
        case BLE_MessageID_t::BLE_ExtraMapData:
        {
            BLE_MapData_Handler(value,BLE_MessageID);
            break;
        }

        default:
        {
        break;
        }
    }

    ui->tableWidge_2->scrollToBottom();
}
/*********************************************************************************************************/
void MainWindow::sendData(){

//    bleConnection.writeData((QString)ui->lineSendDataEdit->text());
}
/*********************************************************************************************************/
void MainWindow::connectDevice()
{
    //    ui->BLE_AutoConnCheckBox->isChecked();
    emit connectToDevice(ui->BLE_DetectedDeviceComboBox->currentIndex());
}
/*********************************************************************************************************/
void MainWindow::on_BLE_DisconnectButton_clicked()
{
    emit DisconnectBLE_Dev();
}

/*********************************************************************************************************/
void MainWindow::InitializeMapGraph(void)
{
    mGraph1 = ui->MapViewWidget->addGraph();

    mGraph1->setScatterStyle(QCPScatterStyle::ssDot);
    mGraph1->setLineStyle(QCPGraph::lsNone);

    ui->MapViewWidget->graph()->setName(QString("New graph %1").arg(ui->MapViewWidget->graphCount()-1));
//    ui->MapViewWidget->graph()->setLineStyle((QCPGraph::LineStyle)(std::rand()%5+1));
//    if (std::rand()%100 > 50)
//        ui->MapViewWidget->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(std::rand()%14+1)));
    QPen graphPen;
    graphPen.setColor(QColor(std::rand()%245+10, std::rand()%245+10, std::rand()%245+10));
    graphPen.setWidthF(std::rand()/(double)RAND_MAX*2+1);
    ui->MapViewWidget->graph()->setPen(graphPen);
    ui->MapViewWidget->replot();
}

/*********************************************************************************************************/
void MainWindow::GraphAppendData(uint32_t X_Pos,uint32_t Y_Pos)
{
    qv_x.append( ((float)X_Pos) /4);
    qv_y.append( ((float)Y_Pos) /4);
    mGraph1->setData(qv_x,qv_y);

    ui->MapViewWidget->xAxis->setRange
            ( *std::min_element(qv_x.begin(),qv_x.end() ) -3,
              *std::max_element(qv_x.begin(),qv_x.end() ) +3);

    ui->MapViewWidget->yAxis->setRange
        (  *std::min_element(qv_y.begin() ,qv_y.end() ) -3,
           *std::max_element(qv_y.begin() ,qv_y.end() ) +10 );

    ui->MapViewWidget->replot();
    ui->MapViewWidget->update();
}

void MainWindow::on_pushButton_3_clicked()
{
    qv_x.clear();
    qv_y.clear();
    ui->MapViewWidget->update();
    ui->MapViewWidget->replot();
}




/*********************************************************************************************************/
void MainWindow::titleDoubleClick(QMouseEvent* event)
{
    Q_UNUSED(event)
    if (QCPTextElement *title = qobject_cast<QCPTextElement*>(sender()))
    {
        // Set the plot title by double clicking on it
        bool ok;
        QString newTitle = QInputDialog::getText(this, "QCustomPlot example", "New plot title:", QLineEdit::Normal, title->text(), &ok);
        if (ok)
        {
            title->setText(newTitle);
            ui->MapViewWidget->replot();
        }
    }
}
/*********************************************************************************************************/
void MainWindow::axisLabelDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
    // Set an axis label by double clicking on it
    if (part == QCPAxis::spAxisLabel) // only react when the actual axis label is clicked, not tick label or axis backbone
    {
        bool ok;
        QString newLabel = QInputDialog::getText(this, "QCustomPlot example", "New axis label:", QLineEdit::Normal, axis->label(), &ok);
        if (ok)
        {
            axis->setLabel(newLabel);
            ui->MapViewWidget->replot();
        }
    }
}
/*********************************************************************************************************/
void MainWindow::legendDoubleClick(QCPLegend *legend, QCPAbstractLegendItem *item)
{
    // Rename a graph by double clicking on its legend item
    Q_UNUSED(legend)
    if (item) // only react if item was clicked (user could have clicked on border padding of legend where there is no item, then item is 0)
    {
        QCPPlottableLegendItem *plItem = qobject_cast<QCPPlottableLegendItem*>(item);
        bool ok;
        QString newName = QInputDialog::getText(this, "QCustomPlot example", "New graph name:", QLineEdit::Normal, plItem->plottable()->name(), &ok);
        if (ok)
        {
            plItem->plottable()->setName(newName);
            ui->MapViewWidget->replot();
        }
    }
}
/*********************************************************************************************************/
void MainWindow::selectionChanged()
{
    /*
   normally, axis base line, axis tick labels and axis labels are selectable separately, but we want
   the user only to be able to select the axis as a whole, so we tie the selected states of the tick labels
   and the axis base line together. However, the axis label shall be selectable individually.

   The selection state of the left and right axes shall be synchronized as well as the state of the
   bottom and top axes.

   Further, we want to synchronize the selection of the graphs with the selection state of the respective
   legend item belonging to that graph. So the user can select a graph by either clicking on the graph itself
   or on its legend item.
  */

    // make top and bottom axes be selected synchronously, and handle axis and tick labels as one selectable object:
    if (ui->MapViewWidget->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->MapViewWidget->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
        ui->MapViewWidget->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->MapViewWidget->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
    {
        ui->MapViewWidget->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
        ui->MapViewWidget->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    }
    // make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
    if (ui->MapViewWidget->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->MapViewWidget->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
        ui->MapViewWidget->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->MapViewWidget->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
    {
        ui->MapViewWidget->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
        ui->MapViewWidget->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    }

    // synchronize selection of graphs with selection of corresponding legend items:
    for (int i=0; i<ui->MapViewWidget->graphCount(); ++i)
    {
        QCPGraph *graph = ui->MapViewWidget->graph(i);
        QCPPlottableLegendItem *item = ui->MapViewWidget->legend->itemWithPlottable(graph);
        if (item->selected() || graph->selected())
        {
            item->setSelected(true);
            graph->setSelection(QCPDataSelection(graph->data()->dataRange()));
        }
    }
}
/*********************************************************************************************************/
void MainWindow::mousePress()
{
    // if an axis is selected, only allow the direction of that axis to be dragged
    // if no axis is selected, both directions may be dragged

    if (ui->MapViewWidget->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui->MapViewWidget->axisRect()->setRangeDrag(ui->MapViewWidget->xAxis->orientation());
    else if (ui->MapViewWidget->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui->MapViewWidget->axisRect()->setRangeDrag(ui->MapViewWidget->yAxis->orientation());
    else
        ui->MapViewWidget->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}
/*********************************************************************************************************/
void MainWindow::mouseWheel()
{
    // if an axis is selected, only allow the direction of that axis to be zoomed
    // if no axis is selected, both directions may be zoomed

    if (ui->MapViewWidget->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui->MapViewWidget->axisRect()->setRangeZoom(ui->MapViewWidget->xAxis->orientation());
    else if (ui->MapViewWidget->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui->MapViewWidget->axisRect()->setRangeZoom(ui->MapViewWidget->yAxis->orientation());
    else
        ui->MapViewWidget->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
}
/*********************************************************************************************************/
void MainWindow::addRandomGraph()
{
    int n = 50; // number of points in graph
    double xScale = (std::rand()/(double)RAND_MAX + 0.5)*2;
    double yScale = (std::rand()/(double)RAND_MAX + 0.5)*2;
    double xOffset = (std::rand()/(double)RAND_MAX - 0.5)*4;
    double yOffset = (std::rand()/(double)RAND_MAX - 0.5)*10;
    double r1 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    double r2 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    double r3 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    double r4 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    QVector<double> x(n), y(n);
    for (int i=0; i<n; i++)
    {
        x[i] = (i/(double)n-0.5)*10.0*xScale + xOffset;
        y[i] = (qSin(x[i]*r1*5)*qSin(qCos(x[i]*r2)*r4*3)+r3*qCos(qSin(x[i])*r4*2))*yScale + yOffset;
    }

    ui->MapViewWidget->addGraph();
    ui->MapViewWidget->graph()->setName(QString("New graph %1").arg(ui->MapViewWidget->graphCount()-1));
    ui->MapViewWidget->graph()->setData(x, y);
    ui->MapViewWidget->graph()->setLineStyle((QCPGraph::LineStyle)(std::rand()%5+1));
    if (std::rand()%100 > 50)
        ui->MapViewWidget->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(std::rand()%14+1)));
    QPen graphPen;
    graphPen.setColor(QColor(std::rand()%245+10, std::rand()%245+10, std::rand()%245+10));
    graphPen.setWidthF(std::rand()/(double)RAND_MAX*2+1);
    ui->MapViewWidget->graph()->setPen(graphPen);
    ui->MapViewWidget->replot();
}
/*********************************************************************************************************/
void MainWindow::removeSelectedGraph()
{
    if (ui->MapViewWidget->selectedGraphs().size() > 0)
    {
        ui->MapViewWidget->removeGraph(ui->MapViewWidget->selectedGraphs().first());
        ui->MapViewWidget->replot();
    }
}
/*********************************************************************************************************/
void MainWindow::removeAllGraphs()
{
    ui->MapViewWidget->clearGraphs();
    ui->MapViewWidget->replot();
}
/*********************************************************************************************************/
void MainWindow::contextMenuRequest(QPoint pos)
{
    QMenu *menu = new QMenu(this);
    menu->setAttribute(Qt::WA_DeleteOnClose);

    if (ui->MapViewWidget->legend->selectTest(pos, false) >= 0) // context menu on legend requested
    {
        menu->addAction("Move to top left", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignLeft));
        menu->addAction("Move to top center", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignHCenter));
        menu->addAction("Move to top right", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignRight));
        menu->addAction("Move to bottom right", this, SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignRight));
        menu->addAction("Move to bottom left", this, SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignLeft));
    } else  // general context menu on graphs requested
    {
        menu->addAction("Add random graph", this, SLOT(addRandomGraph()));
        if (ui->MapViewWidget->selectedGraphs().size() > 0)
            menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
        if (ui->MapViewWidget->graphCount() > 0)
            menu->addAction("Remove all graphs", this, SLOT(removeAllGraphs()));
    }

    menu->popup(ui->MapViewWidget->mapToGlobal(pos));
}
/*********************************************************************************************************/
void MainWindow::moveLegend()
{
    if (QAction* contextAction = qobject_cast<QAction*>(sender())) // make sure this slot is really called by a context menu action, so it carries the data we need
    {
        bool ok;
        int dataInt = contextAction->data().toInt(&ok);
        if (ok)
        {
            ui->MapViewWidget->axisRect()->insetLayout()->setInsetAlignment(0, (Qt::Alignment)dataInt);
            ui->MapViewWidget->replot();
        }
    }
}
/*********************************************************************************************************/
void MainWindow::graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
    // since we know we only have QCPGraphs in the plot, we can immediately access interface1D()
    // usually it's better to first check whether interface1D() returns non-zero, and only then use it.
    double dataValue = plottable->interface1D()->dataMainValue(dataIndex);
    QString message = QString("Clicked on graph '%1' at data point #%2 with value %3.").arg(plottable->name()).arg(dataIndex).arg(dataValue);
    ui->statusbar->showMessage(message, 2500);
}

void MainWindow::on_BLE_BlockSignalsCheckBox_stateChanged(int arg1)
{
    (void)arg1;
    if(ui->BLE_BlockSignalsCheckBox->isChecked()){
        emit BLE_BlockData(true);
    }
    else{
        emit BLE_BlockData(false);
    }
}

