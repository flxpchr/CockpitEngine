#ifndef OGLOVERLAY_H
#define OGLOVERLAY_H

#include <QWidget>
#include <QStackedLayout>
#include <QStackedWidget>

#include <opencv2/opencv.hpp>

#include <widgets/opengloverlay/qoverlaylabel.h>
#include <widgets/opengloverlay/cqtopencvviewergl.h>

class QStackedOverlayGL : public QStackedWidget
{
    Q_OBJECT

public:
    explicit QStackedOverlayGL(QWidget *parent = nullptr);
    ~QStackedOverlayGL();
    void setBackground(const cv::Mat &mat);
    void setToolLocation(bool located,const cv::Point2f &image_pos= cv::Point2f(0,0), const cv::Point2f &projected_pos = cv::Point2f(0,0));
    void paintFrame(bool found, const QPointF &pt_orig, const QPointF &pt_x, const QPointF &pt_y, const QPointF &pt_z);
    void setRawVideoSize(int w,int h);
    void repaint();
    void showRegistrationPattern(std::vector<cv::Point2f> saved_2d_points, std::vector<cv::Point2f> projected_points);
    void hideRegistrationPattern();
    void setArrow(const QPointF &pos,const QPointF &delta);
    QOverlayLabel* overlay();
    CQtOpenCVViewerGl* background();
protected:
    void resizeEvent(QResizeEvent *event);
public slots:
    void updateOverlaySize(int w, int h);
private:
    QOverlayLabel overlay_;
    CQtOpenCVViewerGl background_;
    int videoRawWidth_;
    int videoRawHeight_;
    float videoAspectRatio_, scale_;
};

#endif // OGLOVERLAY_H
