#ifndef TouchRecognizer_H
#define TouchRecognizer_H
#include <QGesture>
#include <QGestureRecognizer>
#include <QEvent>
#include <QWidget>
#include <QTimer>

namespace touch{

    class MyEventHandler : public QObject{
        Q_OBJECT
    public slots:
        void timerEnded(){
            emit tapAndHold2FTriggered();
        }
    signals:
        void tapAndHold2FTriggered();
    };

    class QTouchRecognizer : public QGestureRecognizer{

    public:
        MyEventHandler eventHandler;
        QTouchRecognizer(QWidget* parent, long holdTimeout);
        QGestureRecognizer::Result recognize(QGesture *g, QObject * o, QEvent * e);
        void timerEvent(QTimerEvent *event);

        void timerEnds();
        QGesture* getGesture();
    private:
        long holdTimeout;
        QPointF pt1Start;
        QPointF pt2Start;
        QGesture* gesture;
        void startTimer1();
        void killTimer1();
        void killTimers();
        QTimer *timer1;
        bool timer1Ended;
        void finishGesture(QGesture *g);
    };

}
#endif // TouchRecognizer_H

