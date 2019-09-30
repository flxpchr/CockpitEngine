#include <QPointF>
#include <QLabel>
#include <QLineF>
#include <Qt>
#include <QFont>
#include <QColor>
#include <QGestureEvent>
#include <QSizePolicy>
#include <QGestureRecognizer>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>

#include <widgets/touch/qmovecatcher.h>

namespace touch {
    QMoveCatcher::~QMoveCatcher(){
        QGestureRecognizer::unregisterRecognizer(gh2fType);
        if(gh2f!=nullptr){
            widget_->ungrabGesture(gh2fType);
            widget_->removeEventFilter(touchFilter_);
            delete touchFilter_;
            delete gh2f;
        }
    }

    QMoveCatcher::QMoveCatcher(QWidget *parent):
        QObject(parent),widget_(parent),gh2f(nullptr){
        scribbling=false;
        delta = QPointF();
        dZ = 0;
        deltaThreshold = 30;
        fingerSize = 30;
        radiusSize = fingerSize/2;

        grabMyGestures(widget_);
    }

    void QMoveCatcher::tapAndHold2FTriggered(){
        gh2f->timerEnds();

        QGesture *g=gh2f->getGesture();
        if(g!=nullptr){
            QList<QGesture*> qglist = QList<QGesture*>();
            qglist.append(gh2f->getGesture());
            gestureEvent(new QGestureEvent(qglist));
        }
    }

    void QMoveCatcher::grabMyGestures(QWidget *parent){
        touchFilter_ = new QTouchFilter(this);
        gh2f = new QTouchRecognizer(parent,700);
        QObject::connect(&gh2f->eventHandler,&MyEventHandler::tapAndHold2FTriggered,this,&QMoveCatcher::tapAndHold2FTriggered);
        gh2fType = QGestureRecognizer::registerRecognizer(gh2f);
        widget_->grabGesture(gh2fType);
        widget_->installEventFilter(touchFilter_);
        QGraphicsDropShadowEffect *effect = new QGraphicsDropShadowEffect(widget_);
        effect->setOffset(1,1);
        effect->setBlurRadius(3);
        widget_->setGraphicsEffect(effect);
        //widget_->grabGesture(Qt::TapGesture);
        //widget_->grabGesture(Qt::PanGesture);
        //widget_->grabGesture(Qt::SwipeGesture);
        //widget_->grabGesture(Qt::TapAndHoldGesture);
        //widget_->grabGesture(Qt::PinchGesture);
    }

    bool QMoveCatcher::gestureEvent(QGestureEvent *event){
        if (QGesture *gh2fg = event->gesture(gh2fType)){
            scribbling = gh2fg->property("triggered").toBool();
            //qDebug() << "Gesture Update, gestureEvent :"<< scribbling<<"\t"<<QTime::currentTime().toString("h:mm:ss.z");
            if(scribbling){
                QLineF lStart(gh2fg->property("pt1Start").toPointF(),
                              gh2fg->property("pt2Start").toPointF());
                ptMidStart = lStart.center();

                QLineF lCurrent(gh2fg->property("pt1").toPointF(),
                                gh2fg->property("pt2").toPointF());
                ptMid = lCurrent.center();
                double dist1 = lStart.length();

                delta = ptMid-ptMidStart;
                dZ = dist1-lCurrent.length();

                applyThresholds();
                emit updateDelta(ptMidStart,delta,dZ,dist1);
            }
            else{
                ptMidStart = QPointF();
                pt2Start = QPointF();
                pt1Start = QPointF();
                delta = QPointF();
                dZ = 0;
                emit updateDelta(ptMidStart,delta,dZ,0);
            }
        }
        return true;
    }

    int QMoveCatcher::applyThreshold(int s,double ratio){
        double threshold = deltaThreshold*ratio;
        if(abs(s)<threshold)
            return 0;
        else
            return s>0?s-threshold:s+threshold;
    }

    void QMoveCatcher::applyThresholds(){
        int longueur_2 = pow(delta.x(),2)+pow(delta.y(),2);
        if(longueur_2<pow(deltaThreshold,2)){
            delta=QPointF();
        }
        else{
            double longueur = sqrt(longueur_2);
            double vx = abs(delta.x())/longueur;
            double vy = abs(delta.y())/longueur;

            delta.setX(applyThreshold(delta.x(),vx));
            delta.setY(applyThreshold(delta.y(),vy));
        }

        dZ = applyThreshold(dZ,1);
    }
}


