#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QKeyEvent>
#include <QMainWindow>
#include <iostream>
#include <QObject>
#include <QCloseEvent>
#include <QMessageBox>
#include <QSound>
#include <QMenu>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void keyPressEvent(QKeyEvent* event);
    void closeEvent (QCloseEvent *event);
    void initAlert(){
        mSoundAlert.setLoops(1000);
    }
    void startAlarmFromThread();
    void stopAlarmFromThread();
public slots:
        void startAlarm();
        void stopAlarm();

signals:
    void closeApplication();
    void upEvent();
    void downEvent();
    void rightEvent();
    void leftEvent();
    void upRightEvent();
    void upLeftEvent();
    void downLeftEvent();
    void downRightEvent();
    void plusEvent();
    void minusEvent();
    void stopEvent();
private:
    bool mAlarmPlaying;
    QSound mSoundAlert;
    QMenu *mViewMenu;

};

#endif // MAINWINDOW_HPP
