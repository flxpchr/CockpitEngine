#include <widgets/touch/qmovecatcher2.h>
#include <QDebug>
#include <QVBoxLayout>
#include <QScroller>
#include <QPainter>

namespace touch {
    QMoveCatcher2::QMoveCatcher2(QWidget *parent):QWidget(parent),        
        _rotationAngle(0),
        _scaleFactor(1),
        _currentStepScaleFactor(1),
        _scribbling(false),
        _twoFingerOn(false),
        _firstPressedPoint(0,0),
        _lastPressedPoint(0,0){

    }

    void QMoveCatcher2::emitDelta(){
        const QPointF zeroP(0,0);
        QPointF pos=_firstPressedPoint;
        QPointF delta2D=zeroP;
        qreal dZ = 0;
        qreal dR = 0;
        qreal baseDz = 0;
        if(_twoFingerOn){
            delta2D = zeroP;
            baseDz = _baseDz;
            dZ = _baseDz-_dZ;//_currentStepScaleFactor-1;
            dR = _rotationAngle;
            pos = (_pos1+_pos2)/2;

            if(qIsNaN(dZ)) dZ = 0;
            if(qIsNaN(baseDz)) baseDz = 0;
            if(qIsNaN(dR)) dR = 0;
        }else{
            delta2D = _lastPressedPoint-_firstPressedPoint;
            pos = _firstPressedPoint;
        }

        emit updateDelta(pos, delta2D,dZ,baseDz,dR);
    }


    void QMoveCatcher2::setWidget(QWidget *w){        
        QVBoxLayout *layout = new QVBoxLayout;
        layout->addWidget(w);
        layout->setContentsMargins(0,0,0,0);
        setLayout(layout);
        QScroller::grabGesture(this, QScroller::LeftMouseButtonGesture);
        QList<Qt::GestureType> gestures;
        gestures << Qt::PinchGesture;
        //gestures << Qt::PanGesture;
        //gestures << Qt::TapAndHoldGesture;
        grabGestures(gestures);
        _widget=w;
    }

    void QMoveCatcher2::grabGestures(const QList<Qt::GestureType> &gestures){
        foreach (Qt::GestureType gesture, gestures)
            grabGesture(gesture);
    }

    bool QMoveCatcher2::event(QEvent *event)
    {
        if(event->type() == QEvent::TouchBegin
                ||event->type() == QEvent::TouchUpdate
                ){
            QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);

            _twoFingerOn = (touchEvent->touchPoints().count() == 2);
            if(_twoFingerOn){
                QPointF p1,p2;
                p1 = touchEvent->touchPoints().at(0).pos();
                p2 = touchEvent->touchPoints().at(1).pos();
                qreal dz = QLineF(p1,p2).length();
                if(_pos1.isNull())
                    _pos1 = p1;
                if(_pos2.isNull())
                    _pos2 = p2;
                if(_baseDz==0&&!_pos1.isNull()&&!_pos2.isNull())
                    _baseDz = dz;

                _dZ = dz;

                //qDebug() << "QMoveCatcher2: _pos1: "<<_pos1<<", pos2"<<_pos2
                //                <<"_baseDz"<<_baseDz << ", _dZ: "<<_dZ <<"\n"<<touchEvent;
            }

            //qDebug() << "_twoFingerOn: "<<_twoFingerOn<<"\t"<<touchEvent;

            return true;
        }
        if (event->type() == QEvent::Gesture){

            return gestureEvent(static_cast<QGestureEvent*>(event));
        }
        return QWidget::event(event);
    }

    void QMoveCatcher2::reset(){
        _rotationAngle = 0;
        _scaleFactor = 1;
        _currentStepScaleFactor = 1;
        _firstPressedPoint=QPointF(0,0);
        _lastPressedPoint=QPointF(0,0);
        _baseDz = 0;
        _pos1=QPointF(0,0);
        _pos2=QPointF(0,0);
        _dZ = 0;
    }

    void QMoveCatcher2::mouseDoubleClickEvent(QMouseEvent *)
    {
        reset();
        update();
        emitDelta();
        if(isFullScreen()) {
            showNormal();
         } else {
            showFullScreen();
         }
    }

    //! [gesture event handler]
    bool QMoveCatcher2::gestureEvent(QGestureEvent *event)
    {
        if (QGesture *pan = event->gesture(Qt::PanGesture))
            panTriggered(static_cast<QPanGesture *>(pan));
        else if (QGesture *pinch = event->gesture(Qt::PinchGesture))
            pinchTriggered(static_cast<QPinchGesture *>(pinch));
        else if (QGesture *tapAndHold = event->gesture(Qt::TapAndHoldGesture))
            tapAndHoldTriggered(static_cast<QTapAndHoldGesture *>(tapAndHold));

        return true;
    }

    void QMoveCatcher2::mousePressEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton && !_twoFingerOn) {
            _firstPressedPoint = event->pos();
            _scribbling = true;
        }
    }

    void QMoveCatcher2::mouseMoveEvent(QMouseEvent *event)
    {
        if ((event->buttons() & Qt::LeftButton) && _scribbling && !_twoFingerOn){
            _lastPressedPoint = event->pos();
            if(_firstPressedPoint.isNull())
                _firstPressedPoint = event->pos();
//            qDebug() << "_firstPressedPoint: "<<_firstPressedPoint
//                     <<"\t_lastPressedPoint: "<<_lastPressedPoint;
            update();
            emitDelta();
        }
    }

    void QMoveCatcher2::mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton && _scribbling) {
            _scribbling = false;
            reset();
            update();
            emitDelta();
        }
    }

    void QMoveCatcher2::tapAndHoldTriggered(QTapAndHoldGesture *gesture)
    {
    #ifndef QT_NO_CURSOR
        switch (gesture->state()) {
            case Qt::GestureStarted:
            case Qt::GestureUpdated:
                setCursor(Qt::SizeAllCursor);
                break;
            default:
                setCursor(Qt::ArrowCursor);
        }
    #endif
        //QPointF delta = gesture->hotSpot()-gesture->position();

        update();
        emitDelta();
    }

    //! [gesture event handler]

    void QMoveCatcher2::panTriggered(QPanGesture *gesture)
    {
    #ifndef QT_NO_CURSOR
        switch (gesture->state()) {
            case Qt::GestureStarted:
            case Qt::GestureUpdated:
                setCursor(Qt::SizeAllCursor);
                break;
            case Qt::GestureFinished:
                _twoFingerOn = false;
                break;
            default:
                setCursor(Qt::ArrowCursor);
        }
    #endif
        //QPointF delta = gesture->delta();
        //qCDebug(lcExample) << "panTriggered():" << gesture;

        update();
        emitDelta();
    }

    //! [pinch function]
    void QMoveCatcher2::pinchTriggered(QPinchGesture *gesture)
    {
        QPinchGesture::ChangeFlags changeFlags = gesture->changeFlags();
        if (changeFlags & QPinchGesture::RotationAngleChanged) {
            qreal rotationDelta = gesture->rotationAngle() - gesture->lastRotationAngle();
            _rotationAngle += rotationDelta;
            //qCDebug(lcExample) << "pinchTriggered(): rotate by" << rotationDelta << "->" << rotationAngle;
        }
        if (changeFlags & QPinchGesture::ScaleFactorChanged) {

            _currentStepScaleFactor = gesture->totalScaleFactor();
            //qDebug()  << "pinchTriggered(): zoom by" << gesture->scaleFactor() << "->" << _currentStepScaleFactor;
        }
        if (gesture->state() == Qt::GestureFinished) {
            _scaleFactor *= _currentStepScaleFactor;
            _currentStepScaleFactor = 1;
            reset();
        }
        update();
        emitDelta();
    }
}
