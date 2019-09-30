#include <QDebug>
#include <QTouchEvent>
#include <QTimer>

#include <widgets/touch/qtouchrecognizer.h>
namespace touch {
    /*
     * Touch point states :
     * Qt::TouchPointPressed
     * Qt::TouchPointMoved
     * Qt::TouchPointStationary
     * Qt::TouchPointReleased
     */

    QTouchRecognizer::QTouchRecognizer(QWidget* parent,long holdTimeout):
        QGestureRecognizer(),holdTimeout(holdTimeout){

        timer1= new QTimer(parent);
        timer1->setSingleShot(true);
        QObject::connect(timer1, &QTimer::timeout, &eventHandler, &MyEventHandler::timerEnded);
        timer1Ended = false;
    }



    void QTouchRecognizer::timerEnds(){
        timer1Ended = true;
        gesture->setProperty("triggered", QVariant(true));
    }

    void QTouchRecognizer::startTimer1(){
        timer1Ended = false;
        pt1Start = QPointF();
        pt2Start = QPointF();
        timer1->start(holdTimeout);
    }

    void QTouchRecognizer::killTimer1(){
        if(timer1->isActive()){
            timer1->stop();
        }
        timer1Ended = false;
    }

    void QTouchRecognizer::killTimers(){
        pt1Start = QPointF();
        pt2Start = QPointF();
        killTimer1();
    }

    void QTouchRecognizer::finishGesture(QGesture *g){
        pt1Start=QPointF();
        pt2Start=QPointF();
        g->setProperty("pt2", QVariant());
        g->setProperty("pt1", QVariant());
        g->unsetHotSpot();
        g->setProperty("triggered", QVariant(false));
    }

    QGesture * QTouchRecognizer::getGesture(){
        return gesture;
    }


    QGestureRecognizer::Result QTouchRecognizer::recognize(QGesture *g, QObject *, QEvent * e){
        QTouchEvent* te= nullptr;
        te = dynamic_cast<QTouchEvent*>(e);
        gesture = g;

        if(te!=nullptr){
            if(te->type()==QEvent::TouchCancel){
                killTimer1();
                finishGesture(g);
                return QGestureRecognizer::CancelGesture;
            }
            else if (te->type()==QEvent::TouchEnd){
                killTimer1();
                finishGesture(g);
                return QGestureRecognizer::FinishGesture;
            }
            else if(te->type()==QEvent::TouchBegin)
                killTimers();


            QList<QTouchEvent::TouchPoint> pts = te->touchPoints();
            if (pts.size()>=2){
                QPointF p0 = pts.at(0).pos();
                QPointF p1 = pts.at(1).pos();
                if(pt1Start.isNull()){
                    pt1Start = p0;
                    g->setProperty("pt1Start", QVariant(p0));
                }
                g->setProperty("pt1", QVariant(p0));

                if(pt2Start.isNull()){
                    pt2Start = p1;
                    g->setProperty("pt2Start", QVariant(p1));
                    QPointF mid = QPointF(pt1Start.x()+(pt2Start.x()-pt1Start.x())/2,pt1Start.y()+(pt2Start.y()-pt1Start.y())/2);
                    g->setHotSpot(mid);
                    //qDebug() << "pt1Start : " << pt1Start << "\tpt2Start : " << pt2Start << "\tmid : " << mid;
                }
                g->setProperty("pt2", QVariant(p1));

                if(timer1Ended){
                    g->setProperty("triggered", QVariant(true));
                }
                else{
                    if (pts.at(0).state() == Qt::TouchPointStationary
                            ||pts.at(1).state() == Qt::TouchPointStationary)
                        startTimer1();
                    else
                        killTimer1();
                }

                if (pts.at(0).state() == Qt::TouchPointReleased
                        || pts.at(1).state() == Qt::TouchPointReleased){
                    killTimer1();
                    finishGesture(g);
                    return QGestureRecognizer::CancelGesture;
                }
            }
            else{
                killTimers();
                finishGesture(g);
                return QGestureRecognizer::FinishGesture;
            }

            if(timer1Ended)
                return QGestureRecognizer::TriggerGesture;

            return QGestureRecognizer::MayBeGesture;
        }

        return QGestureRecognizer::Ignore;
    }
}



