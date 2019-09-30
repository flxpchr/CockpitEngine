#include "widgets/calibrationwidget.hpp"

CalibrationWidget::CalibrationWidget(QWidget *parent) : QWidget(parent)
{
    // Title
    QLabel* title = new QLabel("EXTRINSICS CALIBRATIONS",parent);
    title->setAlignment(Qt::AlignTop);
    title->setAlignment(Qt::AlignHCenter);

    // First step
    QHBoxLayout* first_step = new QHBoxLayout(parent);
    QLabel* first_text = new QLabel("1) Click to begin : ",parent);
    start_calib_ = new QPushButton("Let's start !",parent);
    first_step->addWidget(first_text);
    first_step->addWidget(start_calib_);

    // Second step
    QHBoxLayout* second_step = new QHBoxLayout(parent);
    QLabel* second_text = new QLabel("1) Move to the 3 points : ",parent);
    point_6d_done_ = new QPushButton("Get 6D point !",parent);
    second_step->addWidget(second_text);
    second_step->addWidget(point_6d_done_);

    // Third step
    QHBoxLayout* third_step = new QHBoxLayout(parent);
    QLabel* third_text = new QLabel("2) Calibrate the 6D ",parent);
    calibrate_6d_ = new QPushButton("Calibrate 6D !",parent);
    third_step->addWidget(third_text);
    third_step->addWidget(calibrate_6d_);    // Third step

    // Fourth step
    QHBoxLayout* fourth_step = new QHBoxLayout(parent);
    QLabel* fourth_text = new QLabel("3) Move to the 3 points : ",parent);
    point_3d_done_ = new QPushButton("Get 3D point !",parent);
    fourth_step->addWidget(fourth_text);
    fourth_step->addWidget(point_3d_done_);

    // Fifth step
    QHBoxLayout* fifth_step = new QHBoxLayout(parent);
    QLabel* fifth_text = new QLabel("4) Calibrate the 6D ",parent);
    calibrate_3d_ = new QPushButton("Calibrate 3D !",parent);
    fifth_step->addWidget(fifth_text);
    fifth_step->addWidget(calibrate_3d_);

    // Sixth step
    QHBoxLayout* sixth_step = new QHBoxLayout(parent);
    QLabel* sixth_text = new QLabel("5) Calibrate the camera pose ",parent);
    calibrate_camera_ = new QPushButton("Calibrate Cam !",parent);
    sixth_step->addWidget(sixth_text);
    sixth_step->addWidget(calibrate_camera_);


    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addWidget(title);
    vlay->addLayout(first_step);
    vlay->addLayout(second_step);
    vlay->addLayout(third_step);
    vlay->addLayout(fourth_step);
    vlay->addLayout(fifth_step);
    vlay->addLayout(sixth_step);
    setLayout(vlay);

//    vlay->setSpacing(100);
    vlay->setStretch(0,90);

    start_calib_->setEnabled(true);
    point_6d_done_->setEnabled(false);
    point_3d_done_->setEnabled(false);
    calibrate_3d_->setEnabled(false);
    calibrate_6d_->setEnabled(false);
    calibrate_camera_->setEnabled(false);

    // Connect button signal to appropriate slot
    connect(start_calib_, SIGNAL (released()), this, SLOT (activate_calibration()));
}

void CalibrationWidget::activate_calibration(){
    point_6d_done_->setEnabled(true);
    point_3d_done_->setEnabled(true);
}
