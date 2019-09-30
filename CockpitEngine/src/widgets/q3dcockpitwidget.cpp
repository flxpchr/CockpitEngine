#include "widgets/q3dcockpitwidget.hpp"

Q3DCockpitWidget::~Q3DCockpitWidget(){
    delete root_entity_;
    delete container_;
    delete camera_;
}

Q3DCockpitWidget::Q3DCockpitWidget(Qt3DCore::QEntity* root_entity, QWidget *parent) :
    QWidget(parent),container_(parent),root_entity_(root_entity){

    // Camera configuration
    camera_ = view_.camera();
    camera_->lens()->setProjectionType(Qt3DRender::QCameraLens::PerspectiveProjection);
    camera_->setPosition(QVector3D(0, 1.0f, 4.0f));
    camera_->setViewCenter(QVector3D(0, 0, 0));

    // Camera control settings
    QOrbitCameraController *cam_controller = new QOrbitCameraController(root_entity_);
    cam_controller->setLinearSpeed( 3.0f );
    cam_controller->setLookSpeed( 80.0f );
    cam_controller->setCamera(camera_);

    // Configure container_
    view_.setRootEntity(root_entity_);
    container_ = QWidget::createWindowContainer(&view_, this);
    container_->setMaximumSize(view_.screen()->size());
    container_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
}

void Q3DCockpitWidget::show(bool b){
    if(b){
        this->setMinimumSize(QSize(300, 150));
        container_->setVisible(true);
        view_.show();
    }
    else{
        this->setMinimumSize(QSize(0, 0));
        container_->setVisible(false);
        view_.hide();
    }
}

bool Q3DCockpitWidget::isHidden(){
    return !container_->isVisible();
}

void Q3DCockpitWidget::resizeEvent(QResizeEvent *event){
    container_->resize(event->size());
}
