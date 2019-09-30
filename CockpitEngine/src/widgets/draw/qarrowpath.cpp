#include <widgets/draw/qarrowpath.h>

namespace draw {
    QArrowPath::QArrowPath():QPainterPath(){}
    QArrowPath::QArrowPath(QLineF line, int arrowSize):QPainterPath(){

        QPointF speedVector=QPointF(line.dx(),line.dy());
        QPointF p1 = line.p1();
        QPointF p2 = line.p2();

        qreal lineLength = line.length();

        speedVector /= lineLength;
        speedVector *= arrowSize;
        QLineF normLine = line.normalVector().unitVector();
        QPointF normVector = QPointF(normLine.dx(),normLine.dy());
        QPointF normVectorSized = arrowSize*normVector;
        //QPointF normVector = QPointF(-speedVector.y(),speedVector.x());

        moveTo(p1);
        lineTo(p2+arrowSize/5*normVector);
        lineTo(p2+normVectorSized);
        lineTo(p2+arrowSize/20*speedVector);
        lineTo(p2-normVectorSized);
        lineTo(p2-arrowSize/5*normVector);
        lineTo(p1);
        closeSubpath();
    }
}
