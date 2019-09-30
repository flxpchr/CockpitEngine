#ifndef QARROWITEM_H
#define QARROWITEM_H

#include <QObject>
#include <QLineF>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QStyleOptionGraphicsItem>
#include <QPainter>
#include <QWidget>
#include <QBrush>
#include "qarrowpath.h"
namespace draw {
class QGraphicsArrowItem : public QGraphicsLineItem{
public:
    QGraphicsArrowItem();
    QGraphicsArrowItem(const QLineF &line,int arrowSize=10, QGraphicsItem *parent=nullptr);
    void setBrush(QBrush b);
    QBrush getBrush();
    void paint(QPainter * painter, const QStyleOptionGraphicsItem * option,
               QWidget * widget);
    QRectF boundingRect() const;
    void setArrow(const QLineF &line,int s);
private:
    void init();
    QBrush brush;
    QArrowPath arrowPath;
};
}
#endif // QARROWITEM_H
