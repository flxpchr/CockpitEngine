#include "widgets/opengloverlay/qstackedoverlaygl.h"
#include <QDebug>
#include <QResizeEvent>

QStackedOverlayGL::QStackedOverlayGL(QWidget *parent) :
    QStackedWidget(parent),overlay_(this,1920,1080)
{
    QStackedLayout* l = static_cast<QStackedLayout*>(layout());
    l->setStackingMode(QStackedLayout::StackAll);

    background_.setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
    overlay_.setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
    setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);

    addWidget(&overlay_);
    addWidget(&background_);

    adjustSize();

    QObject::connect(&background_,&CQtOpenCVViewerGl::imageSizeChanged,[&](int w, int h){
        overlay_.resize(QSize(w,h));
    });
    std::cout <<"QStackedLayout: "<< width() <<":" << height()<<":" << videoAspectRatio_<<std::endl;
}

void QStackedOverlayGL::setArrow(const QPointF &pos,const QPointF &delta){
    overlay_.setArrow(pos-overlay_.geometry().topLeft(),delta);
}

CQtOpenCVViewerGl* QStackedOverlayGL::background()
{
    return &background_;
}

QOverlayLabel* QStackedOverlayGL::overlay()
{
    return &overlay_;
}

QStackedOverlayGL::~QStackedOverlayGL()
{

}

void QStackedOverlayGL::setBackground(const cv::Mat &mat){
    background_.showImage(mat);
}

void QStackedOverlayGL::setToolLocation(bool located,const cv::Point2f &image_pos, const cv::Point2f &projected_pos){
    overlay_.setToolLocationFromThread(located,image_pos,projected_pos);
    //std::cout << "instrumentTip: " << image_pos << std::endl;
}

void QStackedOverlayGL::paintFrame(bool found, const QPointF &pt_orig, const QPointF &pt_x, const QPointF &pt_y, const QPointF &pt_z){
    overlay_.paintFrame(found, pt_orig, pt_x, pt_y, pt_z);
}

void QStackedOverlayGL::updateOverlaySize(int w, int h){
    setRawVideoSize(w,h);
}

void QStackedOverlayGL::resizeEvent(QResizeEvent *event){
    QSize s = event->size();
    int dx=0;
    int dy=0;
    if(videoRawHeight_ !=0){
        if(s.width()/s.height()>videoRawWidth_/videoRawHeight_){
            int drawing_height = s.height();
            int drawing_width = videoAspectRatio_*drawing_height;
            dx = (s.width()-drawing_width)/2;
            overlay()->setGeometry(dx,dy,drawing_width,drawing_height);
        }
        else{
            int drawing_width = s.width();
            int drawing_height = drawing_width/videoAspectRatio_;
            dy = (s.height()-drawing_height)/2;
            overlay()->setGeometry(dx,dy,drawing_width,drawing_height);
        }
    }
}

void QStackedOverlayGL::setRawVideoSize(int w, int h)
{
    qDebug()<<"Setting raw video size to:"<< w<<", "<< h;
    videoRawWidth_ = w;
    videoRawHeight_ = h;
    videoAspectRatio_ = 1.0*w/h;
    overlay_.setRawVideoSize(w, h);
}

void QStackedOverlayGL::showRegistrationPattern(std::vector<cv::Point2f> saved_2d_points, std::vector<cv::Point2f> projected_points){
  overlay_.showRegistrationPattern(saved_2d_points, projected_points);
}

void QStackedOverlayGL::hideRegistrationPattern(){
  overlay_.hideRegistrationPattern();
}
