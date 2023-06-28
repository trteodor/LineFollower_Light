#ifndef LFSERVICEAPPPLOTS_H
#define LFSERVICEAPPPLOTS_H


#include <QMainWindow>

#include "QWidget"
#include "qcustomplot.h"

class LF_ServiceAppPlot : public QMainWindow
{
    Q_OBJECT
public:
    explicit LF_ServiceAppPlot(QCustomPlot *UIPassedplot, QCPGraph::LineStyle LineStyle);
    ~LF_ServiceAppPlot();

    void LfGraph_Initialize(void);
    void LfGraph_AppendData(float X_Pos,float Y_Pos);


signals:


private slots:
    /******************************************************************************/

    void LfGraph_axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
    void LfGraph_legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
    void LfGraph_selectionChanged();
    void LfGraph_mousePress();
    void LfGraph_mouseWheel();
    void LfGraph_addRandomGraph();
    void LfGraph_removeSelectedGraph();
    void LfGraph_removeAllGraphs();
    void LfGraph_contextMenuRequest(QPoint pos);
    void LfGraph_moveLegend();
    void LfGraph_graphClicked(QCPAbstractPlottable *plottable, int dataIndex);
    /******************************************************************************/


private:
    QCustomPlot *UIplotP;

    QPointer<QCPGraph> Graph1;
    QPointer<QCPGraph> Graph2;

    QVector<double> *DataVector_X, *DataVector_Y;




};





#endif // LFSERVICEAPPPLOTS_H
