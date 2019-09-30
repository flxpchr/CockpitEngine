#include "widgets/joystickwidget.hpp"

JoystickWidget::JoystickWidget(QWidget *parent) : QWidget(parent)
{
    is_recording_ = false;
    ref_is_made_ = false;

    // Title
    QLabel* title = new QLabel("CAMERA JOYSTICK",parent);
    title->setAlignment(Qt::AlignTop);
    title->setAlignment(Qt::AlignHCenter);

    // Initialize buttons
    up_button_ = new QPushButton("UP",parent);
    left_button_ = new QPushButton("LEFT",parent);
    right_button_ = new QPushButton("RIGHT",parent);
    down_button_ = new QPushButton("DOWN",parent);
    stop_button_ = new QPushButton("STOP",parent);
    zoom_plus_button_ = new QPushButton("+",parent);
    zoom_minus_button_ = new QPushButton("-",parent);
    record_button_ = new QPushButton("REC",parent);
    ref_button_ = new QPushButton("REF",parent);
    back_n_forth_button_ = new QPushButton("BACK_N_FORTH",parent);

    up_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
    down_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    record_button_->setEnabled(true);
    ref_button_->setEnabled(true);
    back_n_forth_button_->setEnabled(true);

    // Initialize speed line edit
    speed_spin_ = new QDoubleSpinBox(parent);
    speed_spin_->setDecimals(3);
    speed_spin_->setMaximum(0.05);
    speed_spin_->setMinimum(0.0);
    speed_spin_->setSingleStep(0.001);
    speed_spin_->setValue(0.020);

    // Initialize KP line edit
    KP_ = new QDoubleSpinBox(parent);
    KP_->setDecimals(1);
    KP_->setMaximum(500);
    KP_->setMinimum(0.0);
    KP_->setSingleStep(1);
    KP_->setValue(80.0); //80 good for tangential movements

    // Initialize KD line edit
    KD_ = new QDoubleSpinBox(parent);
    KD_->setDecimals(1);
    KD_->setMaximum(30);
    KD_->setMinimum(0.0);
    KD_->setSingleStep(0.5);
    KD_->setValue(16); // 8 good for tangential movements // 3 demo

    // Initialize KP line edit
    KI_ = new QDoubleSpinBox(parent);
    KI_->setDecimals(1);
    KI_->setMaximum(500);
    KI_->setMinimum(0.0);
    KI_->setSingleStep(1);
    KI_->setValue(90);// 90 good for tangential movements

    // Initialize alpha line edit
    alpha_ = new QDoubleSpinBox(parent);
    alpha_->setDecimals(1);
    alpha_->setMaximum(5000);
    alpha_->setMinimum(0.0);
    alpha_->setSingleStep(1);
    alpha_->setValue(2800); // 5 good

    // Initialize gamma line edit
    gamma_ = new QDoubleSpinBox(parent);
    gamma_->setDecimals(2);
    gamma_->setMaximum(3);
    gamma_->setMinimum(0.0);
    gamma_->setSingleStep(0.01);
    gamma_->setValue(1); //0.04 good

    // Initialize v_limit_ line edit
    v_limit_ = new QDoubleSpinBox(parent);
    v_limit_->setDecimals(3);
    v_limit_->setMaximum(3);
    v_limit_->setMinimum(0.0);
    v_limit_->setSingleStep(0.001);
    v_limit_->setValue(0.01); //0.04 good

    // Left/Right Layout
    QHBoxLayout* left_right_layout = new QHBoxLayout(parent);
    left_right_layout->addWidget(left_button_);
    left_right_layout->addWidget(right_button_);

    // Arrows Layout
    QVBoxLayout* arrows_layout = new QVBoxLayout(parent);
    arrows_layout->addWidget(up_button_);
    arrows_layout->addLayout(left_right_layout);
    arrows_layout->addWidget(down_button_);

    // Zoom Layout
    QVBoxLayout* zoom_layout = new QVBoxLayout(parent);
    zoom_layout->addWidget(zoom_plus_button_);
    zoom_layout->addWidget(zoom_minus_button_);

    // Directions layout
    QHBoxLayout* dir_layout = new QHBoxLayout(parent);
    dir_layout->addLayout(arrows_layout);
    dir_layout->addLayout(zoom_layout);

    // Speed Layout
    QHBoxLayout* speed_layout = new QHBoxLayout(parent);
    QLabel* speed_label = new QLabel("speed [m/s]: ",parent);
    speed_layout->addWidget(speed_label);
    speed_layout->addWidget(speed_spin_);

    // KP Layout
    QHBoxLayout* KP_layout = new QHBoxLayout(parent);
    QLabel* KP_label = new QLabel("KP: ",parent);
    KP_layout->addWidget(KP_label);
    KP_layout->addWidget(KP_);

    // KD Layout
    QHBoxLayout* KD_layout = new QHBoxLayout(parent);
    QLabel* KD_label = new QLabel("KD: ",parent);
    KD_layout->addWidget(KD_label);
    KD_layout->addWidget(KD_);

    // KI Layout
    QHBoxLayout* KI_layout = new QHBoxLayout(parent);
    QLabel* KI_label = new QLabel("KI: ",parent);
    KI_layout->addWidget(KI_label);
    KI_layout->addWidget(KI_);

    // alpha Layout
    QHBoxLayout* alpha_layout = new QHBoxLayout(parent);
    QLabel* alpha_label = new QLabel("alpha: ",parent);
    alpha_layout->addWidget(alpha_label);
    alpha_layout->addWidget(alpha_);

    // gamma Layout
    QHBoxLayout* gamma_layout = new QHBoxLayout(parent);
    QLabel* gamma_label = new QLabel("gamma: ",parent);
    gamma_layout->addWidget(gamma_label);
    gamma_layout->addWidget(gamma_);

    // v_limit_ Layout
    QHBoxLayout* v_limit_layout = new QHBoxLayout(parent);
    QLabel* v_limit_label = new QLabel("v_limit: ",parent);
    v_limit_layout->addWidget(v_limit_label);
    v_limit_layout->addWidget(v_limit_);

    QVBoxLayout* vlay = new QVBoxLayout(parent);
    vlay->addWidget(title);
    vlay->addLayout(dir_layout);
    vlay->addWidget(stop_button_);
//    vlay->addWidget(ref_button_);
    vlay->addWidget(record_button_);
    vlay->addLayout(speed_layout);
    vlay->addLayout(KP_layout);
    vlay->addLayout(KD_layout);
    vlay->addLayout(KI_layout);
    vlay->addLayout(alpha_layout);
    vlay->addLayout(gamma_layout);
    vlay->addLayout(v_limit_layout);
    vlay->addWidget(back_n_forth_button_);
    setLayout(vlay);

//    vlay->setSpacing(100);
//    vlay->setStretch(0,90);

    // Connect button signal to appropriate slot
    connect(stop_button_, SIGNAL (released()), this, SLOT (stop()));
    connect(up_button_, SIGNAL (released()), this, SLOT (move_up()));
    connect(left_button_, SIGNAL (released()), this, SLOT (move_left()));
    connect(right_button_, SIGNAL (released()), this, SLOT (move_right()));
    connect(down_button_, SIGNAL (released()), this, SLOT (move_down()));
    connect(zoom_minus_button_, SIGNAL (released()), this, SLOT (zoom_out()));
    connect(zoom_plus_button_, SIGNAL (released()), this, SLOT (zoom_in()));
    connect(record_button_, SIGNAL (released()), this, SLOT (record()));
    connect(ref_button_, SIGNAL (released()), this, SLOT (make_ref()));
    connect(back_n_forth_button_, SIGNAL (released()), this, SLOT (back_n_forth()));
}

void JoystickWidget::stop(){
    up_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
    down_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    back_n_forth_button_->setEnabled(true);
}

void JoystickWidget::move_up(){
    up_button_->setEnabled(false);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
    down_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(true);
    stop_button_->setEnabled(true);
}

void JoystickWidget::move_left(){
    up_button_->setEnabled(true);
    left_button_->setEnabled(false);
    right_button_->setEnabled(true);
    down_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(true);
    stop_button_->setEnabled(true);
}

void JoystickWidget::move_right(){
    up_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(false);
    down_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(true);
    stop_button_->setEnabled(true);
}

void JoystickWidget::move_down(){
    up_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
    down_button_->setEnabled(false);
    zoom_minus_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(true);
    stop_button_->setEnabled(true);
}

void JoystickWidget::zoom_in(){
    up_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
    down_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(true);
    zoom_plus_button_->setEnabled(false);
    stop_button_->setEnabled(true);
}

void JoystickWidget::zoom_out(){
    up_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
    down_button_->setEnabled(true);
    zoom_minus_button_->setEnabled(false);
    zoom_plus_button_->setEnabled(true);
    stop_button_->setEnabled(true);
}

void JoystickWidget::back_n_forth(){
    up_button_->setEnabled(false);
    left_button_->setEnabled(false);
    right_button_->setEnabled(false);
    down_button_->setEnabled(false);
    zoom_minus_button_->setEnabled(false);
    zoom_plus_button_->setEnabled(false);
    stop_button_->setEnabled(true);
    back_n_forth_button_->setEnabled(false);

    // X seconds in, X seconds out, stop
    emit zoom_plus_button_->released();

    int seconds = 3;
    QTimer::singleShot(1000*seconds, [=]() {emit zoom_minus_button_->released(); } );
    QTimer::singleShot(2*1000*seconds, [=]() {emit stop_button_->released(); } );
}

void JoystickWidget::record(){
    if(is_recording_){
        is_recording_ = false;
        record_button_->setText("REC");
    }else{
        is_recording_ = true;
        record_button_->setText("STOP REC");
    }
}

void JoystickWidget::make_ref(){
    if(ref_is_made_){
        ref_is_made_ = false;
        ref_button_->setText("REF");
    }else{
        ref_is_made_ = true;
        ref_button_->setText("DEL REF");
    }
}
