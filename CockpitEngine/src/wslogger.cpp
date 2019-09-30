#include "wslogger.h"

#include <QStringBuilder>
#include <QTimer>

WSLogger::WSLogger(const QUrl &url, bool debug, QObject *parent) :
    QObject(parent),
    m_url(url),
    m_debug(debug)
{
    if (m_debug)
        qDebug() << "WebSocket server:" << url;
    connect(&m_webSocket, &QWebSocket::connected, this, &WSLogger::onConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &WSLogger::closed);
    connect(this, &WSLogger::_sendMsgSignal, this, &WSLogger::_sendMsg);
    m_webSocket.open(QUrl(url));
}

void WSLogger::onConnected()
{
    if (m_debug)
        qDebug() << "WebSocket connected";

}

void WSLogger::sendMsg(const QString &key, float value){
    using namespace std::chrono;
    static std::chrono::high_resolution_clock::time_point initial_time = high_resolution_clock::now();

     auto now =  high_resolution_clock::now();
     const float t = duration_cast< duration<float>>( now - initial_time ).count() ;
     QString str = key+QString(":")
             + QString::number(t)+":"
             + QString::number(value);

    emit _sendMsgSignal(str);
}

void WSLogger::_sendMsg(const QString &value){
     if (m_debug)
        qDebug() << value;
     m_webSocket.sendTextMessage(value);
}
