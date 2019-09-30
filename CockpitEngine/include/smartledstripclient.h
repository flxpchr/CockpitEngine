#ifndef SMARTLEDSTRIPCLIENT_H
#define SMARTLEDSTRIPCLIENT_H

#include <QObject>
#include <QTimer>
#include <QtNetwork>
#include <QDebug>
#include <QSerialPort>
#include <iostream>

#include <virtuose_interface.hpp>

namespace slsc {
    enum State
    {
       PowerOff,
       PowerOn,
       TrocarFound,
       Locked,
       LockedBlink
    };

    class SmartLedStripInterface : public QObject{
        Q_OBJECT
    public:
        SmartLedStripInterface(QObject *parent = nullptr):SmartLedStripInterface(0,parent){}
        SmartLedStripInterface(int repeatEveryMs,QObject *parent = nullptr):QObject(parent),mState(PowerOff),isBlinkMode(false){
            connect(&mRepeatTimer, &QTimer::timeout, this, &SmartLedStripInterface::update);
            repeat(repeatEveryMs);
        }
        ~SmartLedStripInterface(){
            mRepeatTimer.stop();
            disconnect(&mRepeatTimer, &QTimer::timeout, this, &SmartLedStripInterface::update);
        }

        void repeat(int reapeatTime){
            if(mRepeatTimer.isActive())
                mRepeatTimer.stop();
            if(reapeatTime>0)
                mRepeatTimer.start(reapeatTime);
        }
        void setTimeout(int timeout){
            sendCmd("timeout",QString::asprintf("%d",timeout));
        }
        void setBlinkDuration(int duration){
            sendCmd("blinkDuration",QString::asprintf("%d",duration));
        }
        void setBlinkMode(bool b){isBlinkMode=b;}

        void setState(State s){
            if(s!=mState||!mRepeatTimer.isActive()){
                mState = s;
                update();
            }
        }
        State getState() {return mState;}
    public slots:
        virtual void sendCmd(QString path,QString value=nullptr)=0;
    protected slots:
        void update(){
            QString mode = isBlinkMode?"blink":"rgb";
            switch (mState) {
                case State::PowerOff:
                    emitCmd(mode,"000000");
                    break;
                case State::PowerOn:
                    emitCmd(mode,"cc2000");
                    break;
                case State::TrocarFound:
                    emitCmd(mode,"005500");
                    break;
                case State::Locked:
                    emitCmd(mode,"550000");
                    break;
                case State::LockedBlink:

                    emitCmd("blink","550000");
                    break;
                default:
                    break;


            }
        }
    protected:
        State mState;
        QTimer mRepeatTimer;
        bool isBlinkMode;
    private:
        void emitCmd(QString path, QString value=nullptr){
            QGenericReturnArgument ret;
            QMetaObject::invokeMethod(this, "sendCmd", Qt::ConnectionType::QueuedConnection,
                                      ret,
                                      Q_ARG(QString, path), Q_ARG(QString, value));
        }
        QString getMode(){
            return isBlinkMode?"blink":"rgb";
        }

    signals:
        void response(QString s);
        void error(QString s);
    };

    class SmartLedStrip3DMedical : public SmartLedStripInterface
    {
        Q_OBJECT
    public:
        SmartLedStrip3DMedical(VirtuoseInterface *r, QObject *parent = nullptr):SmartLedStripInterface(parent),
        _robot(r){

        }
    public slots:
        void sendCmd(QString path,QString value=nullptr) override{
            QString str = QString("/%1/%2\n").arg(path).arg(value);
            bool valid;
            //Rouge
            float red256, green256, blue256;
            red256 = (float)value.mid(0,2).toInt(&valid,16);
            if(valid)
                green256 = (float)value.mid(2,2).toInt(&valid,16);
            if(valid)
                blue256 = (float)value.mid(4,2).toInt(&valid,16);
            if(valid){
                _colors[0] = red256/256.0*0.96;
                _colors[1] = green256/256.0*0.96;
                _colors[2] = blue256/256.0*0.96;

//                qDebug() <<  _colors[0] << "\t"
//                        <<  _colors[1] << "\t"
//                        <<  _colors[2] << "\t"
//                        <<  _colors[3] << "\t"
//                        <<  _colors[4] << "\t"
//                        <<  _colors[5] << "\t"
//                        <<  _colors[6];

                _robot->setPwmsOutput(_colors);
            }
        }
    private:
        float _colors[7]{0.,0.,0.,0.,0.,0.,0.};
        VirtuoseInterface *_robot;
    };

    class SmartLedStripHttpClient : public SmartLedStripInterface
    {
        Q_OBJECT
    public:
        SmartLedStripHttpClient(QObject *parent = nullptr):SmartLedStripInterface(parent){
            connect(&mNam, &QNetworkAccessManager::finished,this, &SmartLedStripHttpClient::wsFinished);
        }
        SmartLedStripHttpClient(QUrl url, int repeat):SmartLedStripInterface(repeat){
            mUrl = url;
        }
        ~SmartLedStripHttpClient() override {
            disconnect(&mNam, &QNetworkAccessManager::finished, this, &SmartLedStripHttpClient::wsFinished);
        }

        void setUrl(QUrl url){
            mUrl = url;
        }
    public slots:
        void sendCmd(QString path,QString value=nullptr) override{
            QString urlStr = mUrl.toString()+"/"+path;
            //qDebug() << "Sending: " << urlStr<<", value :"<<value;
            QUrl url(urlStr);
            if(value!=nullptr){
                QUrlQuery query;
                query.addQueryItem("value", value);
                url.setQuery(query);
            }
            QNetworkRequest req(url);
            mNam.get(req);
        }
    protected slots:
        void wsFinished(QNetworkReply* rep){
            QByteArray bts = rep->readAll();
            QString str(bts);
            if(rep->error()==QNetworkReply::NetworkError::NoError)
                emit response(str);
            else
                emit error(str);
        }
    private:
        QNetworkAccessManager mNam;
        QUrl mUrl;
    };

    class SmartLedStripSerialClient:public SmartLedStripInterface{
        Q_OBJECT
    public:
        SmartLedStripSerialClient(QObject *parent = nullptr):SmartLedStripInterface(parent){
            mTimer.setSingleShot(true);
            connect(&mSerialPort, &QSerialPort::bytesWritten, this, &SmartLedStripSerialClient::handleBytesWritten);
            connect(&mSerialPort, &QSerialPort::errorOccurred, this, &SmartLedStripSerialClient::handleError);
            connect(&mTimer, &QTimer::timeout, this, &SmartLedStripSerialClient::handleTimeout);
        }
        SmartLedStripSerialClient(int repeat, QObject *parent = nullptr):SmartLedStripInterface(repeat, parent){
            mTimer.setSingleShot(true);
            connect(&mSerialPort, &QSerialPort::bytesWritten, this, &SmartLedStripSerialClient::handleBytesWritten);
            connect(&mSerialPort, &QSerialPort::errorOccurred, this, &SmartLedStripSerialClient::handleError);
            connect(&mTimer, &QTimer::timeout, this, &SmartLedStripSerialClient::handleTimeout);
        }
        ~SmartLedStripSerialClient() override {
            if(mSerialPort.isOpen())
                mSerialPort.close();
            disconnect(&mSerialPort,  &QSerialPort::bytesWritten, this, &SmartLedStripSerialClient::handleBytesWritten);
            disconnect(&mSerialPort, &QSerialPort::errorOccurred, this, &SmartLedStripSerialClient::handleError);
            disconnect(&mTimer, &QTimer::timeout, this, &SmartLedStripSerialClient::handleTimeout);
        }

        SmartLedStripSerialClient(const QString& serialPortName,int repeat,  QObject *parent = nullptr):SmartLedStripSerialClient(repeat,parent){
            open(serialPortName);
        }
        void open(const QString& serialPortName){
            mSerialPort.setPortName(serialPortName);
            mSerialPort.setBaudRate(QSerialPort::Baud115200);
            mSerialPort.open(QIODevice::WriteOnly);
        }

        void write(const QByteArray &writeData){
            if(!mSerialPort.isOpen())
                return;

            mWriteData = writeData;

            const qint64 bytesWritten = mSerialPort.write(writeData);

            if (bytesWritten == -1) {
                emit error( QObject::tr("Failed to write the data to port %1, error: %2")
                                    .arg(mSerialPort.portName())
                                    .arg(mSerialPort.errorString())
                                 );
            } else if (bytesWritten != mWriteData.size()) {
                emit error( QObject::tr("Failed to write all the data to port %1, error: %2")
                                    .arg(mSerialPort.portName())
                                    .arg(mSerialPort.errorString())
                                 );
            }

            mTimer.start(5000);
        }
    public slots:
        void sendCmd(QString path,QString value=nullptr) override{
            QString str = QString("/%1/%2\n").arg(path).arg(value);
            write(str.toLocal8Bit());
        }
    private slots:
        void handleBytesWritten(qint64 bytes){
            mBytesWritten += bytes;
            if (mBytesWritten == mWriteData.size()) {
                QString msg=QString("Written to %1 : %2").arg(mSerialPort.portName()).arg(QString(mWriteData));
                emit response(msg);
                mBytesWritten = 0;
            }
        }
        void handleTimeout(){
            emit error(QObject::tr("Operation timed out for port %1, error: %2")
                                .arg(mSerialPort.portName())
                                .arg(mSerialPort.errorString()));
        }
        void handleError(QSerialPort::SerialPortError err){
            if (err == QSerialPort::WriteError) {
                emit error( QObject::tr("An I/O error occurred while writing"
                                                " the data to port %1, error: %2")
                                    .arg(mSerialPort.portName())
                                    .arg(mSerialPort.errorString()));
            }
        }
    private:
        QSerialPort mSerialPort;
        QByteArray mWriteData;
        qint64 mBytesWritten = 0;
        QTimer mTimer;
    };
}


#endif // SMARTLEDSTRIPCLIENT_H
