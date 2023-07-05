#ifndef GENERICLFQCP_H
#define GENERICLFQCP_H


#include <QMainWindow>

#include "QWidget"
#include "qcustomplot.h"

class GenericLfQCP : public QMainWindow
{
    Q_OBJECT
public:
    explicit GenericLfQCP(void);
    ~GenericLfQCP();

    void LfGraphInitialize(QCustomPlot *UIPassedplot,QCPGraph::LineStyle LineStyle);
    void LfGraph_AppendData(float X_Pos,float Y_Pos);
    void LfGraph_UpdateReplot(void);
    void LfGraph_ClearData(void);

    QCustomPlot *UIplotP;



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


    QPointer<QCPGraph> Graph1;
    QPointer<QCPGraph> Graph2;

    QVector<double> DataVector_X, DataVector_Y;




};





#endif // GENERICLFQCP_H
