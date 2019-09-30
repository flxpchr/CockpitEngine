#include <widgets/draw/qgraphicsdiffcircleitem.h>
#include <widgets/draw/qgraphicsarrowitem.h>
#include <widgets/draw/qarrowpath.h>

namespace draw {

    QGraphicsDiffCircleItem::QGraphicsDiffCircleItem(QPointF c,int baseValue):
        QGraphicsEllipseItem(c.x()-baseValue/4,c.y()-baseValue/4,baseValue/2,baseValue/2){
        mCenterp = c;
        mReverted = 1;
        setBaseValue(baseValue);
    }

    void QGraphicsDiffCircleItem::setBaseValue(int baseValue){
        this->baseValue = baseValue;        
        arrowSize = 50;
        setRect(mCenterp.x()-baseValue/4,mCenterp.y()-baseValue/4,baseValue/2,baseValue/2);
    }

    void QGraphicsDiffCircleItem::setValue(int value){
        this->mValue = value;
        int radiusBase = baseValue/2;
        mAbsDeltaZ = radiusBase-value;

        mRadius = mAbsDeltaZ/2;
        if(value<0)
            mReverted = 1;
        else
            mReverted = -1;
    }

    void QGraphicsDiffCircleItem::paint(QPainter *painter,
                                        const QStyleOptionGraphicsItem *option,
                                        QWidget *widget = nullptr){
        painter->setRenderHints( QPainter::HighQualityAntialiasing);

        QColor curColor;
        QGraphicsEllipseItem::paint(painter,option,widget);

        int radius = mRadius;
        if(radius<80)
            radius=80;
        int absDeltaZ = mAbsDeltaZ;
        if(absDeltaZ<160)
            absDeltaZ=160;

        if(mValue!=0){
            if(mValue>0)
                curColor=QColor(255,0,0,150);
            else
                curColor=QColor(0,255,0,150);

            QPointF left = mCenterp + QPointF(-radius*mReverted,0);
            QPointF top = mCenterp + QPointF(0,radius*mReverted);
            QPointF right = mCenterp + QPointF(radius*mReverted,0);
            QPointF bottom = mCenterp + QPointF(0,-radius*mReverted);

            QLineF line;
            int tmpArrowSize = arrowSize;

            if(mValue>0)
                tmpArrowSize = 0.9*tmpArrowSize;
            line.setPoints(left,left+QPointF(-tmpArrowSize,0));
            painter->fillPath(QArrowPath(line,tmpArrowSize),QBrush(curColor));
            line.setPoints(top,top+QPointF(0,tmpArrowSize));
            painter->fillPath(QArrowPath(line, tmpArrowSize),QBrush(curColor));
            line.setPoints(right,right+QPointF(tmpArrowSize,0));
            painter->fillPath(QArrowPath(line,tmpArrowSize),QBrush(curColor));
            line.setPoints(bottom,bottom+QPointF(0,-tmpArrowSize));
            painter->fillPath(QArrowPath(line, tmpArrowSize),QBrush(curColor));

            painter->setPen(QPen(curColor));
            QRect rect = QRect( mCenterp.x()-radius,mCenterp.y()-radius,absDeltaZ,absDeltaZ);
            painter->drawEllipse(rect);


        }
    }

    QRectF QGraphicsDiffCircleItem::boundingRect() const
    {
        int expended = baseValue + mRadius;
        if(mReverted<0)
            return QGraphicsEllipseItem::boundingRect();
        else
            return QRectF(mCenterp.x()-expended/4,mCenterp.y()-expended/4,expended/2,expended/2);

    }

    int QGraphicsDiffCircleItem::getBaseValue() const
    {
        return baseValue;
    }

    int QGraphicsDiffCircleItem::getValue() const
    {
        return mValue;
    }

    void QGraphicsDiffCircleItem::applyRatio(double r)
    {
        mValue=r*mValue;
        baseValue=r*baseValue;
        mCenterp = r*mCenterp;
    }

    void QGraphicsDiffCircleItem::revertLine(QLine &l){
        l.setPoints(l.p2(),l.p1());
    }
}
