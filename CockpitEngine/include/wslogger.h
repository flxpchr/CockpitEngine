#ifndef WSLOGGER_H
#define WSLOGGER_H

#include <QObject>
#include <QtWebSockets/QWebSocket>
#include <chrono>
#include <QTimer>

class WSLogger: public QObject
{
    Q_OBJECT
public:
    explicit WSLogger(const QUrl &url, bool debug = false, QObject *parent = Q_NULLPTR);
    void sendMsg(const QString &key, float value);
Q_SIGNALS:
    void closed();
    void _sendMsgSignal(const QString &str);

private Q_SLOTS:
    void _sendMsg(const QString &str);
    void onConnected();

private:
    QWebSocket m_webSocket;
    QUrl m_url;
    bool m_debug;
    QTimer m_timer;

};

#endif // WSLOGGER_H
