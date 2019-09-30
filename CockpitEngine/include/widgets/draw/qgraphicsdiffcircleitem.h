#ifndef QGRAPHICSZSPEEDITEM_H
#define QGRAPHICSZSPEEDITEM_H

#include <QObject>
#include <QGraphicsEllipseItem>
#include <QPointF>
#include <QStyleOptionGraphicsItem>
#include <QPainter>
#include <QWidget>
namespace draw {
class QGraphicsDiffCircleItem : public QGraphicsEllipseItem{
    public:
        QGraphicsDiffCircleItem(QPointF mCenterp,int baseValue);
        void setBaseValue(int baseValue);
        void setValue(int mValue);
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                                      QWidget *widget);
        QRectF boundingRect() const;
        int getBaseValue() const;
        int getValue() const;
        void applyRatio(double r);

private:
        void revertLine(QLine &l);
        int baseValue;
        int mValue;
        int arrowSize;
        QPointF mCenterp;
        int mReverted;
        int mRadius;
        int mAbsDeltaZ;
};
}
#endif // QGRAPHICSZSPEEDITEM_H
