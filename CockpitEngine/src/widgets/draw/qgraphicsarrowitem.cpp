#include <widgets/draw/qgraphicsarrowitem.h>
namespace draw {
    QGraphicsArrowItem::QGraphicsArrowItem():
        QGraphicsLineItem(), arrowPath(QGraphicsLineItem::line()){
        init();
    }

    QGraphicsArrowItem::QGraphicsArrowItem(const QLineF &line, int arrowSize, QGraphicsItem *parent):
        QGraphicsLineItem(line, parent),arrowPath(line, arrowSize){
        init();
    }

    void QGraphicsArrowItem::init(){
        setBrush(QBrush(Qt::black));
    }

    void QGraphicsArrowItem::setBrush(QBrush b){
        brush = b;
    }

    QBrush QGraphicsArrowItem::getBrush(){
        return brush;
    }

    void QGraphicsArrowItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr){
        QGraphicsLineItem::paint(painter,option,widget);
        painter->setRenderHints( QPainter::HighQualityAntialiasing);

        QPen pen = QGraphicsLineItem::pen();
        pen.color().setAlpha(100);
        painter->setPen(pen);


        painter->fillPath(arrowPath, brush);
        painter->drawPath(arrowPath);
    }

    QRectF QGraphicsArrowItem::boundingRect() const
    {
        return arrowPath.boundingRect();
    }

    void QGraphicsArrowItem::setArrow(const QLineF &line,int s)
    {
        arrowPath = QArrowPath(line, s);
    }
}
