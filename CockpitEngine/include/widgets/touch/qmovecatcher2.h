#ifndef QMOVECATCHER2_H
#define QMOVECATCHER2_H

#include <QtWidgets>
#include <QWidget>
#include <QPoint>
#include <QGestureEvent>

namespace touch {


class QMoveCatcher2 : public QWidget{
    Q_OBJECT
public:
    QMoveCatcher2(QWidget *parent = nullptr);
    void grabGestures(const QList<Qt::GestureType> &gestures);
    void setWidget(QWidget *w);
protected:
    bool event(QEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
signals:
    void updateDelta(QPointF pos, QPointF delta2D,double dZ, double _baseDz, double dR);
private:
    void emitDelta();
    void reset();
    bool gestureEvent(QGestureEvent *event);
    void panTriggered(QPanGesture*);
    void pinchTriggered(QPinchGesture*);
    void tapAndHoldTriggered(QTapAndHoldGesture*);
    QPointF _firstPressedPoint,_lastPressedPoint;
    bool _scribbling;
    bool _twoFingerOn;

    QPointF _pos1;
    QPointF _pos2;
    qreal _baseDz;
    qreal _dZ;
    qreal _rotationAngle;
    qreal _scaleFactor;
    qreal _currentStepScaleFactor;
    QWidget* _widget;

};

}

#endif // QMOVECATCHER2_H
