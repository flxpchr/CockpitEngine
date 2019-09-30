#ifndef QOVERLAYLABEL_H
#define QOVERLAYLABEL_H

#include <QObject>
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QLabel>
#include <QPainter>
#include <QMetaType>
#include "../draw/qgraphicsdiffcircleitem.h"
#include "../draw/qgraphicsarrowitem.h"

#include <opencv2/opencv.hpp>


class QOverlayLabel : public QGraphicsView{
    Q_OBJECT
public:
    QOverlayLabel(QWidget *parent, int videoWidth=1920, int videoHeight=1080);
    //void paintEvent(QPaintEvent*);
    void resizeEvent(QResizeEvent *event);
    void paintFrame(bool found, const QPointF &pt_orig, const QPointF &pt_x, const QPointF &pt_y, const QPointF &pt_z);
    void setRawVideoSize(int w, int h);
    void showRegistrationPattern(std::vector<cv::Point2f> saved_2d_points, std::vector<cv::Point2f> projected_points);
    void hideRegistrationPattern();
    void setArrow(const QPointF &pos,const QPointF &delta);
    void setGeometry(int x, int y, int w, int h);
    void setDelta2DFromThread(QPointF baseP,QPointF delta2D,double dz);
    void setToolLocationFromThread(bool located, const cv::Point2d &image_pos, const cv::Point2d &projected_pos);
public slots:
    void setToolLocation(bool located, const QPointF &image_pos, const QPointF &projected_pos = QPointF(0,0));
    void setDelta(QPointF pos, QPointF delta2D,double dZ, double baseDz, double dR);
    void repaint();
    void setLogMsg(const QString &s);
private:
    static const QPen RED_PEN;
    static const QPen GREEN_PEN;
    static const QPen ARROW_PEN;
    static const QPen WDASHED_PEN;
    const QPointF m_zeroPoint{0,0};

    QGraphicsRectItem m_rec;
    QGraphicsRectItem m_limitLow;
    QGraphicsRectItem m_limitHigh;
    QGraphicsScene m_scene;
    QGraphicsTextItem m_titleItem;
    QGraphicsTextItem m_logItem;
    QGraphicsEllipseItem m_ToolItem;

    double m_scale;
    double m_limitScaleLow;
    double m_limitScaleHigh;
    QPointF m_deltaLayout;
    int m_videoRawWidth;
    int m_videoRawHeight;
    float m_videoAspectRatio;
    float m_currentRatio;
    QPointF m_toolPos, tool_projected_pos_,m_testPos;
    bool m_toolLocated;

    bool show_pattern_;

    std::vector<cv::Point2f> projected_points_;
    std::vector<cv::Point2f> saved_2d_points_;

    bool calibration_frame_found_;
    QPointF calib_frame_orig_, calib_frame_pt_x_, calib_frame_pt_y_, calib_frame_pt_z_;
    draw::QGraphicsDiffCircleItem m_touchCircle;
    draw::QGraphicsArrowItem m_touchArrow;
};

#endif // QOVERLAYLABEL_H
