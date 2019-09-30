#include "mainwindow.hpp"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),mSoundAlert("alarm.wav"), mAlarmPlaying(false)
{
    setWindowState(Qt::WindowMaximized);

    mViewMenu = new QMenu(this) ;
    mViewMenu->addMenu(QObject::tr("&View"));

    //        .menuBar()->addMenu(QObject::tr("&View"));
}

MainWindow::~MainWindow()
{
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    QMessageBox::StandardButton resBtn = QMessageBox::question( this, "EXIT",
                                                                tr("Are you sure you want to quit ?\n"),
                                                                QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes) {
        event->ignore();
    } else {
        event->accept();
        emit closeApplication();
    }
}

void MainWindow::startAlarm(){
    if(!mAlarmPlaying){
        mAlarmPlaying = true;
        mSoundAlert.play();
    }
}

void MainWindow::stopAlarm(){
    if(mAlarmPlaying)
        mSoundAlert.stop();
    mAlarmPlaying = false;
}

void MainWindow::startAlarmFromThread(){
    if(!mAlarmPlaying)
        QMetaObject::invokeMethod(this, "startAlarm"
                                  ,Qt::ConnectionType::QueuedConnection
                                  );
}

void MainWindow::stopAlarmFromThread(){
    if(mAlarmPlaying)
        QMetaObject::invokeMethod(this, "stopAlarm"
                                  ,Qt::ConnectionType::QueuedConnection
                                  );
}



void MainWindow::keyPressEvent( QKeyEvent* event ) {
    switch ( event->key() ) {
    case Qt::Key_8:
        emit upEvent();
        break;
    case Qt::Key_2:
        emit downEvent();
        break;
    case Qt::Key_4:
        emit leftEvent();
        break;
    case Qt::Key_6:
        emit rightEvent();
        break;
    case Qt::Key_9:
        emit upRightEvent();
        break;
    case Qt::Key_7:
        emit upLeftEvent();
        break;
    case Qt::Key_1:
        emit downLeftEvent();
        break;
    case Qt::Key_3:
        emit downRightEvent();
        break;
    case Qt::Key_5:
        emit stopEvent();
        break;
    case Qt::Key_Plus:
        emit plusEvent();
        break;
    case Qt::Key_Minus:
        emit minusEvent();
        break;
    default:
        event->ignore();
        break;
    }
}
