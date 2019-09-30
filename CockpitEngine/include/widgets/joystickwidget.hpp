#ifndef JOYSTICKWIDGET_HPP
#define JOYSTICKWIDGET_HPP

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <iostream>
#include <QTimer>

class JoystickWidget : public QWidget
{
    Q_OBJECT
public:
    explicit JoystickWidget(QWidget *parent = nullptr);

    QPushButton *up_button_, *down_button_, *left_button_, *right_button_, *stop_button_, *zoom_plus_button_, *zoom_minus_button_, *record_button_, *ref_button_, *back_n_forth_button_;
    QDoubleSpinBox *speed_spin_ , *KP_, *KD_, *KI_, *alpha_, *gamma_, *v_limit_;
private:
    bool is_recording_;
    bool ref_is_made_;

signals:

public slots:
    void stop();
    void move_up();
    void move_down();
    void move_left();
    void move_right();
    void zoom_in();
    void zoom_out();
    void record();
    void make_ref();
    void back_n_forth();
};

#endif // JOYSTICKWIDGET_HPP
