#ifndef Q3DCOCKPITWIDGET_HPP
#define Q3DCOCKPITWIDGET_HPP

#include <QWidget>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DCore/QTransform>

#include <QSettings>
#include <QQueue>
#include <QVector>
#include <Qt3DExtras/QOrbitCameraController>
#include <QScreen>
#include <QResizeEvent>

using namespace Qt3DRender;
using namespace Qt3DExtras;


class Q3DCockpitWidget : public QWidget
{
    Q_OBJECT
public:
    explicit Q3DCockpitWidget(Qt3DCore::QEntity* root_entity, QWidget *parent = nullptr);
    ~Q3DCockpitWidget() override;
    bool isHidden();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    QWidget* container_;
    Qt3DWindow view_;
    QCamera* camera_;
    Qt3DCore::QEntity* root_entity_;

public slots:
    void show(bool b);
};

#endif // Q3DCOCKPITWIDGET_HPP
