#ifndef CALIBRATIONWIDGET_HPP
#define CALIBRATIONWIDGET_HPP

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>

class CalibrationWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CalibrationWidget(QWidget *parent = nullptr);

    QPushButton *start_calib_, *point_6d_done_, *point_3d_done_;
    QPushButton *calibrate_6d_, *calibrate_3d_, *calibrate_camera_;

signals:

public slots:
    void activate_calibration();
};

#endif // CALIBRATIONWIDGET_HPP
