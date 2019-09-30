#ifndef MOVECATCHER_H
#define MOVECATCHER_H

#include <QObject>
#include <QLabel>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QMouseEvent>
#include <QPointF>
#include <QtGui>
#include <QGraphicsLineItem>
#include <QGraphicsPixmapItem>
#include <QLabel>
#include <QSplitter>
#include <QTapAndHoldGesture>
#include <QMainWindow>
#include <QGraphicsEffect>
#include <QWidget>
#include "qtouchrecognizer.h"

class QGestureEvent;
class QPinchGesture;


namespace touch {
    class QTouchFilter;

    class QTestEffect:public QGraphicsEffect{
    public :
        QTestEffect(QWidget *o):QGraphicsEffect(o), w_(o){

        }

    protected:
        QWidget *w_;
        void draw(QPainter *painter)
        {
            static const QPen redPen = QPen(QColor(250,0,0,200),3);
            painter->setPen(redPen);
            painter->drawRect(10,10,w_->width()-10,w_->height()-10);

        }
    };

    class QMoveCatcher : public QObject{
        Q_OBJECT
        friend class QTouchFilter;
    public:
        explicit QMoveCatcher(QWidget *parent = nullptr);
        ~QMoveCatcher();
        QPointF delta;
        double dZ;
        void repaint(int x, int y, int w, int h);
    signals:
        void updateDelta(QPointF pos, QPointF delta2D,double dZ, double baseDz);
    public slots:
        //void mousePressEvent(QMouseEvent * e);
        //void mouseReleaseEvent(QMouseEvent * e);
        //void mouseDoubleClickEvent(QMouseEvent * e);
        //void mouseMoveEvent(QMouseEvent * e);
        void tapAndHold2FTriggered();

    private:
        QWidget* widget_;
        QTouchFilter* touchFilter_;
        int deltaThreshold;
        int fingerSize;
        int radiusSize;
        int arrowSize;
        QPointF pt1Start,pt2Start,ptMidStart,ptMid, pt1, pt2;
        bool scribbling;
        QTouchRecognizer* gh2f;
        Qt::GestureType gh2fType;

        int applyThreshold(int s,double ratio);
        void applyThresholds();
        bool gestureEvent(QGestureEvent *event);
        /*void panTriggered(QPanGesture*);
        void pinchTriggered(QPinchGesture*);
        void swipeTriggered(QSwipeGesture*);*/
        void tapAndHoldTriggered(QTapAndHoldGesture *);
        void grabMyGestures(QWidget *w);
    };

    class QTouchFilter : public QObject
    {
    public:
        QTouchFilter(QMoveCatcher* mc):QObject(),mc_(mc){}
    protected:
        bool eventFilter(QObject *obj, QEvent *event){
            QWidget* widget = static_cast<QWidget*>(obj);
            QEvent::Type t = event->type();
            if (t == QEvent::Gesture)
                   return mc_->gestureEvent(static_cast<QGestureEvent*>(event));

            return QObject::eventFilter(widget, event);
        }
    private:
        QMoveCatcher* mc_;
    };
}



#endif // MOVECATCHER_H
