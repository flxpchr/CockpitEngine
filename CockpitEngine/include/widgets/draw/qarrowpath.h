#ifndef QARROWPATH_H
#define QARROWPATH_H

#include <QPainterPath>

namespace draw {
    class QArrowPath : public QPainterPath
    {
    public:
        QArrowPath();
        QArrowPath(QLineF line, int arrowSize=10);
    };
}


#endif // QARROWPATH_H
