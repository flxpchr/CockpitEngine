#include <QResizeEvent>
#include <QPaintEvent>
#include <QDebug>
#include <QPointF>
#include <QRectF>
#include "widgets/opengloverlay/qoverlaylabel.h"

const QPen QOverlayLabel::RED_PEN= QPen(QColor(250,0,0,200), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
const QPen QOverlayLabel::GREEN_PEN = QPen(QColor(0,250,0,150));
const QPen QOverlayLabel::ARROW_PEN = QPen(QColor(250,250,250,150), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
const QPen QOverlayLabel::WDASHED_PEN = QPen(QColor(250,250,250,200),1,Qt::DashLine);


QOverlayLabel::QOverlayLabel(QWidget *parent, int videoWidth, int videoHeight):
    QGraphicsView(parent),
    m_toolLocated(false),
    m_videoRawWidth(videoWidth),
    m_videoRawHeight(videoHeight),
    calibration_frame_found_(false),
    m_touchCircle(m_zeroPoint,0),
    m_scale(1),
    m_rec(0,0,videoWidth,videoHeight),
    m_limitScaleLow(0.15),
    m_limitScaleHigh(0.7),
    m_limitLow(),
    m_limitHigh(),
    m_ToolItem(-50,-50,50,50)
{
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setStyleSheet("background-color: rgba(0, 0, 0, 0);");
    setScene(&m_scene);

    m_titleItem.setDefaultTextColor(QColor::fromRgb(250,250,250));
    m_titleItem.setFont(QFont("Verdana",15));
    m_titleItem.setPlainText(""); //Titre

    m_logItem.setPlainText("info");
    m_logItem.setDefaultTextColor(QColor::fromRgb(250,250,250));
    m_logItem.setFont(QFont("Verdana",10));

    m_rec.setPen(RED_PEN);
    m_limitLow.setPen(RED_PEN);
    m_limitHigh.setPen(RED_PEN);
    m_ToolItem.setPen(RED_PEN);

    m_touchCircle.setPen(WDASHED_PEN);
    m_touchArrow.setPen(ARROW_PEN);
    m_touchArrow.setBrush(QBrush(QColor(0,0,0,150)));
    m_ToolItem.setBrush(QBrush(QColor(0,0,0,150)));

    m_scene.addItem(&m_rec);
    m_scene.addItem(&m_limitLow);
    m_scene.addItem(&m_limitHigh);
    m_scene.addItem(&m_logItem);

    m_scene.addItem(&m_titleItem);
    m_scene.addItem(&m_touchCircle);
    m_scene.addItem(&m_touchArrow);
    m_scene.addItem(&m_ToolItem);

    m_rec.setZValue(0);
    m_limitLow.setZValue(0);
    m_limitHigh.setZValue(0);
    m_logItem.setZValue(1);
    m_titleItem.setZValue(1);
    m_touchCircle.setZValue(3);
    m_touchArrow.setZValue(4);
    m_ToolItem.setZValue(5);

    show_pattern_ = false;
    setAttribute( Qt::WA_TransparentForMouseEvents );
    show();
}

void QOverlayLabel::repaint(){
    QGraphicsView::repaint(0,0,width(),height());
}

void QOverlayLabel::setGeometry(int x, int y, int w, int h){
    m_deltaLayout = QPointF(x,y);
    QGraphicsView::setGeometry( x, y,  w, h);
}
/*
void QOverlayLabel::setZoom(const QPointF &center,int baseZ, delta){

}*/



void QOverlayLabel::setToolLocationFromThread(bool located, const cv::Point2d &image_pos, const cv::Point2d &projected_pos){
    //std::cout << "instrumentTip: " << image_pos << std::endl;
    QMetaObject::invokeMethod(this, "setToolLocation"
                              ,Qt::ConnectionType::QueuedConnection
                              ,Q_ARG(bool, located)
                              ,Q_ARG(const QPointF &, QPointF(image_pos.x,image_pos.y))
                              ,Q_ARG(const QPointF &, QPointF(projected_pos.x,projected_pos.y)));

}

void QOverlayLabel::setToolLocation(bool located,
                                    const QPointF &image_pos,
                                    const QPointF &projected_pos){
    m_toolLocated = located;
    m_toolPos = image_pos;
    tool_projected_pos_ = projected_pos;
    //qDebug()<< "located: "<<m_toolLocated<< "\tpos: "<< m_toolPos;
    if(m_toolLocated)
        m_ToolItem.setPos(m_toolPos);
    else
        m_ToolItem.setPos(QPointF());
    setLogMsg(QString::asprintf("tool: %.0f,%.0f", m_ToolItem.x(),m_ToolItem.y()));
    repaint();
}

void QOverlayLabel::setDelta2DFromThread(QPointF baseP,QPointF delta2D,double dz){
    baseP = QPointF(width()/2,height()/2);
    QMetaObject::invokeMethod(this, "setDelta"
                              ,Qt::ConnectionType::QueuedConnection
                              ,Q_ARG(QPointF, baseP)
                              ,Q_ARG(QPointF, delta2D)
                              ,Q_ARG(double, dz)
                              ,Q_ARG(double, 0)
                              ,Q_ARG(double, 0));
    repaint();
}

void QOverlayLabel::setDelta(QPointF pos, QPointF delta2D,
                             double dZ, double baseDz, double dR){
    //QString s = QString::asprintf("Delta: %.0f,%.0f,%.0f",delta2D.x(),delta2D.y(), dZ);
    //m_logItem.setPlainText(s);
    if(pos.isNull())
        m_rec.setPen(RED_PEN);
    else
        m_rec.setPen(GREEN_PEN);

    QPointF scaledPos = pos/m_scale;
    QPointF scaledDelta2D = delta2D/m_scale;
    m_touchCircle.setPos(pos/m_scale);
    m_touchCircle.setBaseValue(baseDz/m_scale);
    m_touchCircle.setValue(dZ/m_scale);
    //touchCircle.resetMatrix();
    //touchCircle.setScale(1/m_scale);

    m_touchArrow.setArrow(QLineF(m_zeroPoint,scaledDelta2D),30);
    m_touchArrow.setPos(scaledPos-geometry().topLeft());
    //touchArrow.resetMatrix();
    //touchArrow.setScale(1/m_scale);

    setLogMsg(QString::asprintf("pos: %.2f,%.2f, delta2d: %.2f,%.2f, dZ: %.2f, baseDz: %.2f, dR: %.2f, tool: %.0f,%.0f"
                                ,pos.x(),pos.y()
                                ,delta2D.x(),delta2D.y()
                                ,dZ, baseDz, dR,
                                m_toolPos.x(),m_toolPos.y()));

}

void QOverlayLabel::setArrow(const QPointF &pos,const QPointF &delta){
    m_touchArrow.setArrow(QLineF(pos,pos+delta),40);
    repaint();
}

void QOverlayLabel::resizeEvent(QResizeEvent *){
    double w = parentWidget()->width();
    double h = parentWidget()->height();
    double hFromRatio = w/m_videoAspectRatio;
    double wFromRatio = h*m_videoAspectRatio;
    if(w==wFromRatio && h==hFromRatio)
        return;

    double wOut, hOut;
    if(w-wFromRatio > h-hFromRatio){
        wOut = wFromRatio;
        hOut = h;
    }else{
        wOut = w;
        hOut = hFromRatio;
    }

    double posX=0.0;
    double posY=0.0;
    if(w>wOut)
        posX=(w-wOut)/2.0;
    if(h>hOut)
        posY=(h-hOut)/2.0;


    double wRatio = wOut/m_videoRawWidth;
    //double hRatio = hOut/m_videoRawHeight;

    resetMatrix();
    m_scale = wRatio;

    //TO-do : here is a hack to get back circle as circle...
    scale(wRatio, wRatio);
    m_scene.setSceneRect(0, 0, m_videoRawWidth,  m_videoRawHeight);


    setGeometry(posX, posY, wOut, hOut);

    setLogMsg(QString::asprintf("%.0f/%d, %.0f,%d, pos: %.0f,%.0f, scale: %.2f, lowRect: %.0f,%.0f",wOut,m_videoRawWidth,hOut,m_videoRawHeight,posX, posY,m_scale, m_limitLow.rect().width(),m_limitLow.rect().height()));

    //centerOn(QPointF(m_videoRawWidth/2.0,m_videoRawHeight/2.0));
    repaint();
}

void QOverlayLabel::paintFrame(bool found, const QPointF &pt_orig, const QPointF &pt_x, const QPointF &pt_y, const QPointF &pt_z){
    calibration_frame_found_ = found;
    calib_frame_orig_ = pt_orig;
    calib_frame_pt_x_ = pt_x;
    calib_frame_pt_y_ = pt_y;
    calib_frame_pt_z_ = pt_z;
}

void QOverlayLabel::setRawVideoSize(int w, int h)
{
    m_videoRawWidth = w;
    m_videoRawHeight = h;
    m_videoAspectRatio = 1.0*m_videoRawWidth/m_videoRawHeight;
    m_currentRatio = 1.0*width()/m_videoRawWidth;

    m_scene.setSceneRect(0, 0, m_videoRawWidth,  m_videoRawHeight);
    qreal titleW =m_titleItem.boundingRect().width();
    const QPointF tPos(m_videoRawWidth/2-titleW/2,0);
    m_titleItem.setPos(tPos);
    m_logItem.setPos(QPointF(0,m_videoRawHeight-m_logItem.boundingRect().height()));
    m_limitLow.setRect(m_videoRawWidth*m_limitScaleLow*.5,
                       m_videoRawHeight*m_limitScaleLow*.5*m_videoAspectRatio,
                       m_videoRawWidth-m_videoRawWidth*m_limitScaleLow,
                       m_videoRawHeight-m_videoRawHeight*m_limitScaleLow*m_videoAspectRatio);
    m_limitHigh.setRect(m_videoRawWidth*m_limitScaleHigh*.5,
                        m_videoRawHeight*m_limitScaleHigh*.5*m_videoAspectRatio,
                        m_videoRawWidth-m_videoRawWidth*m_limitScaleHigh,
                        m_videoRawHeight-m_videoRawHeight*m_limitScaleHigh*m_videoAspectRatio);
}

void QOverlayLabel::showRegistrationPattern(std::vector<cv::Point2f> saved_2d_points, std::vector<cv::Point2f> projected_points){
    projected_points_ = projected_points;
    saved_2d_points_ = saved_2d_points;
    show_pattern_ = true;
}

void QOverlayLabel::hideRegistrationPattern(){
    show_pattern_ = false;
}

void QOverlayLabel::setLogMsg(const QString &s)
{
    m_logItem.setPlainText(s);
}
/*
void QOverlayLabel::resizeEvent(QResizeEvent *e){
    QGraphicsView::resizeEvent(e);

    m_currentRatio = 1.0*width()/m_videoRawWidth;
}
*/
/*
void QOverlayLabel::paintEvent(QPaintEvent* )
{
    QPainter p(this);

    float precision_pix_size = 2.0f;

    if(m_toolLocated)
        p.setPen(Qt::darkGreen);
    else
        p.setPen(Qt::red);
    p.drawRect(1,1,width()-2, height()-2);

    if(m_toolLocated){
        p.drawLine(m_toolPos.x()*m_currentRatio-precision_pix_size*m_currentRatio,
                   m_toolPos.y()*m_currentRatio,
                   m_toolPos.x()*m_currentRatio+precision_pix_size*m_currentRatio,
                   m_toolPos.y()*m_currentRatio);
        p.drawLine(m_toolPos.x()*m_currentRatio,
                   m_toolPos.y()*m_currentRatio-precision_pix_size*m_currentRatio,
                   m_toolPos.x()*m_currentRatio,
                   m_toolPos.y()*m_currentRatio+precision_pix_size*m_currentRatio);

        p.setPen(Qt::red);
        p.drawEllipse(tool_projected_pos_*m_currentRatio,
                      precision_pix_size*m_currentRatio,precision_pix_size*m_currentRatio);
    }
    if(calibration_frame_found_){
        p.setPen(Qt::red);
        p.drawLine(calib_frame_orig_*m_currentRatio, calib_frame_pt_x_*m_currentRatio);
        p.setPen(Qt::green);
        p.drawLine(calib_frame_orig_*m_currentRatio, calib_frame_pt_y_*m_currentRatio);
        p.setPen(Qt::blue);
        p.drawLine(calib_frame_orig_*m_currentRatio, calib_frame_pt_z_*m_currentRatio);
    }

    if(show_pattern_){
        // Draw saved 2d points
        p.setPen(Qt::green);
        for(int i=0;i<saved_2d_points_.size();++i){
              p.drawEllipse(QPointF(saved_2d_points_[i].x*m_currentRatio, saved_2d_points_[i].y*m_currentRatio),
                            precision_pix_size*m_currentRatio,precision_pix_size*m_currentRatio);
            }
        }

        // Draw projected points
        if(projected_points_.size() >=6){
            p.setPen(Qt::blue);
            for(int i=0;i<projected_points_.size();++i){
                p.drawRect(QRectF(QPointF(projected_points_[i].x*m_currentRatio-precision_pix_size*m_currentRatio, projected_points_[i].y*m_currentRatio-precision_pix_size*m_currentRatio),
                              QSizeF(2*precision_pix_size*m_currentRatio,2*precision_pix_size*m_currentRatio)));
        }
    }

    if(!m_touchArrow.isEmpty()){
        p.setPen(WDASHED_PEN);
        p.setPen(ARROW_PEN);
        p.setBrush(QBrush(QColor(0,0,0,150)));
        p.drawPath(m_touchArrow);
    }

}
*/


