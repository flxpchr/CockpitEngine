#ifndef KEYPRESSEATER_HPP
#define KEYPRESSEATER_HPP

#include <QObject>
#include <QKeyEvent>

class KeyPressEater : public QObject
{
    Q_OBJECT
protected:
    bool KeyPressEater::eventFilter(QObject *obj, QEvent *event){
        if (event->type() == QEvent::KeyPress) {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            if(keyEvent->isAutoRepeat())
                event->ignore();
            else{
                if(keyEvent->key() == Qt::Key_N){
                    emit keyNPressed(true);
                    event->accept();
                    return true;
                }
                if(keyEvent->key() == Qt::Key_J){
                    emit keyJPressed(true);
                    event->accept();
                    return true;
                }
            }
        }
        if (event->type() == QEvent::KeyRelease) {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            if(keyEvent->isAutoRepeat())
                event->ignore();
            else{
                if(keyEvent->key() == Qt::Key_N){
                    emit keyNPressed(false);
                    event->accept();
                    return true;
                }
                if(keyEvent->key() == Qt::Key_J){
                    emit keyJPressed(false);
                    event->accept();
                    return true;
                }
            }
        }
        return QObject::eventFilter(obj, event);
    }
signals:
    void keyNPressed(bool);
    void keyJPressed(bool);
};

#endif // KEYPRESSEATER_HPP
