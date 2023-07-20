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
    void LfGraph_AppendData(float X_Pos1,float Y_Pos1,
                            float X_Pos2=0,float Y_Pos2=0,
                            float X_Pos3=0,float Y_Pos3=0,
                            float X_Pos4=0,float Y_Pos4=0,
                            float X_Pos5=0,float Y_Pos5=0,
                            float X_Pos6=0,float Y_Pos6=0);
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
    QPointer<QCPGraph> Graph3;
    QPointer<QCPGraph> Graph4;
    QPointer<QCPGraph> Graph5;
    QPointer<QCPGraph> Graph6;


    QPointer<QCPGraph> SelectedPointMarkerGraphY;
    QPointer<QCPGraph> SelectedPointMarkerGraphX;

    QVector<double> DataVector_X1, DataVector_Y1;
    QVector<double> DataVector_X2, DataVector_Y2;
    QVector<double> DataVector_X3, DataVector_Y3;
    QVector<double> DataVector_X4, DataVector_Y4;
    QVector<double> DataVector_X5, DataVector_Y5;
    QVector<double> DataVector_X6, DataVector_Y6;





};





#endif // GENERICLFQCP_H
