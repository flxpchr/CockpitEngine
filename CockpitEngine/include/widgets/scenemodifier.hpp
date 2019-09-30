#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include <QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QNormalDiffuseSpecularMapMaterial>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QPointLight>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#define ANGLE_THRESH    (.0000001f)

const float PI = 3.14159265359f;
const float ARROW_LENGTH = 0.50f;
const float ARROW_RADIUS = ARROW_LENGTH/20;
const QMatrix4x4 TO_Z_UP_QMAT(0,1,0,0,
                              0,0,1,0,
                              1,0,0,0,
                              0,0,0,1);

class SceneModifier : public QObject
{
    Q_OBJECT
public:
    inline QMatrix4x4 eigenToQt(const Eigen::Matrix4f &transform )
    {
        return QMatrix4x4(transform.data()).transposed();
    }

    SceneModifier(Qt3DCore::QEntity* root){
        // Add initial Qt3D root frame
        entity_map_.insert(std::make_pair(QString("root_y_up").toStdString(),root));

        // Add corrected (z up) root frame
        Qt3DCore::QEntity* root_z_up = new Qt3DCore::QEntity(root);
        Qt3DCore::QTransform* to_z_up_tf = new Qt3DCore::QTransform();
        to_z_up_tf->setMatrix(TO_Z_UP_QMAT);
        root_z_up->addComponent(to_z_up_tf);
        entity_map_.insert(std::make_pair(QString("root").toStdString(),root_z_up));

        qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");
        qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");
        QObject::connect(this, SIGNAL(signalPlane(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)),
                         this, SLOT(drawPlane(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)));
        QObject::connect(this, SIGNAL(signalBox(QString,QString,Eigen::Matrix4f,QColor,bool,float,float,float)),
                         this, SLOT(drawBox(QString,QString,Eigen::Matrix4f,QColor,bool,float,float,float)));
        QObject::connect(this, SIGNAL(signalSphere(QString,QString,Eigen::Matrix4f,QColor,bool,float)),
                         this, SLOT(drawSphere(QString,QString,Eigen::Matrix4f,QColor,bool,float)));
        QObject::connect(this, SIGNAL(signalCylinder(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)),
                         this, SLOT(drawCylinder(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)));
        QObject::connect(this, SIGNAL(signalCone(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)),
                         this, SLOT(drawCone(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)));
        QObject::connect(this, SIGNAL(signalArrow(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)),
                         this, SLOT(drawArrow(QString,QString,Eigen::Matrix4f,QColor,bool,float,float)));
        QObject::connect(this, SIGNAL(signal3DAxes(QString,QString,Eigen::Matrix4f,bool,float)),
                         this, SLOT(draw3DAxes(QString,QString,Eigen::Matrix4f,bool,float)));
        QObject::connect(this, SIGNAL(signalVector(QString,QString,Eigen::Vector3f,Eigen::Vector3f,QColor,bool,float)),
                         this, SLOT(drawVector(QString,QString,Eigen::Vector3f,Eigen::Vector3f,QColor,bool,float)));
    }

    ~SceneModifier(){
        for(std::map<std::string, Qt3DCore::QTransform*>::iterator itr = tf_map_.begin(); itr != tf_map_.end(); itr++)
            delete (itr->second);
        tf_map_.clear();
        for(std::map<std::string, Qt3DExtras::QPhongMaterial*>::iterator itr = material_map_.begin(); itr != material_map_.end(); itr++)
            delete (itr->second);
        material_map_.clear();
        for(std::map<std::string, Qt3DCore::QEntity*>::iterator itr = entity_map_.begin(); itr != entity_map_.end(); itr++)
            delete (itr->second);
        entity_map_.clear();
        for(std::map<std::string, Qt3DExtras::QSphereMesh*>::iterator itr = sphere_mesh_map_.begin(); itr != sphere_mesh_map_.end(); itr++)
            delete (itr->second);
        sphere_mesh_map_.clear();
        for(std::map<std::string, Qt3DExtras::QCylinderMesh*>::iterator itr = cylinder_mesh_map_.begin(); itr != cylinder_mesh_map_.end(); itr++)
            delete (itr->second);
        cylinder_mesh_map_.clear();
        for(std::map<std::string, Qt3DExtras::QConeMesh*>::iterator itr = cone_mesh_map_.begin(); itr != cone_mesh_map_.end(); itr++)
            delete (itr->second);
        cone_mesh_map_.clear();
        for(std::map<std::string, Qt3DExtras::QCuboidMesh*>::iterator itr = box_mesh_map_.begin(); itr != box_mesh_map_.end(); itr++)
            delete (itr->second);
        box_mesh_map_.clear();
        for(std::map<std::string, Qt3DExtras::QPlaneMesh*>::iterator itr = plane_mesh_map_.begin(); itr != plane_mesh_map_.end(); itr++)
            delete (itr->second);
        plane_mesh_map_.clear();

    }
public:
    void emitPlane(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float height = 1.0f, float width = 1.0f){
        emit signalPlane(entity, name, pose, c , visible, height, width);
    }
    void emitBox(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float x_size = 1.0f, float y_size = 1.0f, float z_size = 1.0f){
        emit signalBox(entity, name, pose, c, visible, x_size, y_size, z_size);
    }
    void emitSphere(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.1f){
        emit signalSphere(entity, name, pose, c, visible, radius);
    }
    void emitCylinder(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.03f, float length = 0.5f){
        emit signalCylinder(entity, name, pose, c, visible, radius, length);
    }
    void emitCone(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.16f, float length = 0.5f){
        emit signalCone(entity, name, pose, c, visible, radius, length);
    }
    void emitArrow(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = ARROW_RADIUS, float length = ARROW_LENGTH){
        emit signalArrow(entity, name, pose, c, visible, radius, length);
    }
    void emit3DAxes(QString entity, QString name, Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(), bool visible = true, float scale = 1.0f){
        emit signal3DAxes(entity, name, pose, visible, scale);
    }
    void emitVector(QString entity, QString name, Eigen::Vector3f position, Eigen::Vector3f value, QColor c = QColor("blue"), bool visible = true, float radius = ARROW_RADIUS){
        emit signalVector(entity, name, position, value, c, visible, radius);
    }

    bool findEntity(QString name){
        std::map<std::string,Qt3DCore::QEntity*>::iterator it;
        it = entity_map_.find(name.toStdString());
        return !(it == entity_map_.end());
    }

    bool getEntity(QString name, Qt3DCore::QEntity*& entity){
        if(findEntity(name)){
            entity = entity_map_[name.toStdString()];
            return true;
        }else
            return false;
    }

    std::vector<std::string> getEntityNames(){
        std::vector<std::string> name_list;
        for(std::map<std::string,Qt3DCore::QEntity*>::iterator it = entity_map_.begin(); it != entity_map_.end(); ++it)
            name_list.push_back(it->first);
        return name_list;
    }

signals:
    void signalPlane(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float height = 1.0f, float width = 1.0f);
    void signalBox(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float x_size = 1.0f, float y_size = 1.0f, float z_size = 1.0f);
    void signalSphere(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.1f);
    void signalCylinder(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.03f, float length = 0.5f);
    void signalCone(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.16f, float length = 0.5f);
    void signalArrow(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = ARROW_RADIUS, float length = ARROW_LENGTH);
    void signal3DAxes(QString entity, QString name, Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(), bool visible = true, float scale = 1.0f);
    void signalVector(QString entity, QString name, Eigen::Vector3f position, Eigen::Vector3f value, QColor c = QColor("blue"), bool visible = true, float radius = ARROW_RADIUS);

private slots:
    void drawPlane(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float height = 1.0f, float width = 1.0f){
        if(!findEntity(name)){
            Qt3DCore::QEntity *parent;
            if(!getEntity(entity, parent)){
                std::cout << "COULD NOT FIND PARENT ENTITY "<< entity.toStdString()<<" TO DRAW PLANE "<< name.toStdString() << std::endl;
                return;
            }

            // object does not exist yet
            Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
            material->setDiffuse(c);
            material->setEnabled(visible);
            material_map_.insert(std::make_pair(name.toStdString(),material));

            Qt3DCore::QTransform* tf = new Qt3DCore::QTransform();
            QMatrix4x4 correction;
            correction.rotate(90.0f, QVector3D(1,0,0));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf->setMatrix(pose_qmat*correction);
            tf_map_.insert(std::make_pair(name.toStdString(),tf));

            Qt3DExtras::QPlaneMesh* mesh = new Qt3DExtras::QPlaneMesh();
            mesh->setHeight(height);
            mesh->setWidth(width);
            plane_mesh_map_.insert(std::make_pair(name.toStdString(),mesh));

            Qt3DCore::QEntity *planeEntity = new Qt3DCore::QEntity(parent);
            planeEntity->addComponent(mesh);
            planeEntity->addComponent(tf);
            planeEntity->addComponent(material);
            entity_map_.insert(std::make_pair(name.toStdString(),planeEntity));
        }else{
            // object exists
            QMatrix4x4 correction;
            correction.rotate(90.0f, QVector3D(1,0,0));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf_map_[name.toStdString()]->setMatrix(pose_qmat*correction);
            material_map_[name.toStdString()]->setDiffuse(c);
            material_map_[name.toStdString()]->setEnabled(visible);
            plane_mesh_map_[name.toStdString()]->setHeight(height);
            plane_mesh_map_[name.toStdString()]->setWidth(width);
        }
    }

    void drawBox(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float x_size = 1.0f, float y_size = 1.0f, float z_size = 1.0f){
        if(!findEntity(name)){
            Qt3DCore::QEntity *parent;
            if(!getEntity(entity, parent)){
                std::cout << "COULD NOT FIND PARENT ENTITY "<< entity.toStdString()<<" TO DRAW BOX "<< name.toStdString() << std::endl;
                return;
            }

            // object does not exist yet
            Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
            material->setDiffuse(c);
            material->setEnabled(visible);
            material_map_.insert(std::make_pair(name.toStdString(),material));

            Qt3DCore::QTransform* tf = new Qt3DCore::QTransform();
            QMatrix4x4 correction;
            correction.translate(QVector3D(0, 0,z_size/2));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf->setMatrix(pose_qmat*correction);
            tf_map_.insert(std::make_pair(name.toStdString(),tf));

            Qt3DExtras::QCuboidMesh* mesh = new Qt3DExtras::QCuboidMesh();
            mesh->setXExtent(x_size);
            mesh->setYExtent(y_size);
            mesh->setZExtent(z_size);
            box_mesh_map_.insert(std::make_pair(name.toStdString(),mesh));

            Qt3DCore::QEntity *sphereEntity = new Qt3DCore::QEntity(parent);
            sphereEntity->addComponent(mesh);
            sphereEntity->addComponent(tf);
            sphereEntity->addComponent(material);
            entity_map_.insert(std::make_pair(name.toStdString(),sphereEntity));
        }else{
            // object exists
            QMatrix4x4 correction;
            correction.translate(QVector3D(0, 0,z_size/2));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf_map_[name.toStdString()]->setMatrix(pose_qmat*correction);
            material_map_[name.toStdString()]->setDiffuse(c);
            material_map_[name.toStdString()]->setEnabled(visible);
            box_mesh_map_[name.toStdString()]->setXExtent(x_size);
            box_mesh_map_[name.toStdString()]->setYExtent(y_size);
            box_mesh_map_[name.toStdString()]->setZExtent(z_size);
        }
    }

    void drawSphere(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.1f){
        if(!findEntity(name)){
            Qt3DCore::QEntity *parent;
            if(!getEntity(entity, parent)){
                std::cout << "COULD NOT FIND PARENT ENTITY "<< entity.toStdString()<<" TO DRAW SPHERE "<< name.toStdString() << std::endl;
                return;
            }

            // object does not exist yet
            Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
            material->setDiffuse(c);
            material->setEnabled(visible);
            material_map_.insert(std::make_pair(name.toStdString(),material));

            Qt3DCore::QTransform* tf = new Qt3DCore::QTransform();
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf->setMatrix(pose_qmat);
            tf_map_.insert(std::make_pair(name.toStdString(),tf));

            Qt3DExtras::QSphereMesh* mesh = new Qt3DExtras::QSphereMesh();
            mesh->setRadius(radius);
            sphere_mesh_map_.insert(std::make_pair(name.toStdString(),mesh));

            Qt3DCore::QEntity *sphereEntity = new Qt3DCore::QEntity(parent);
            sphereEntity->addComponent(mesh);
            sphereEntity->addComponent(tf);
            sphereEntity->addComponent(material);
            entity_map_.insert(std::make_pair(name.toStdString(),sphereEntity));
        }else{
            // object exists
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf_map_[name.toStdString()]->setMatrix(pose_qmat);
            material_map_[name.toStdString()]->setDiffuse(c);
            material_map_[name.toStdString()]->setEnabled(visible);
            sphere_mesh_map_[name.toStdString()]->setRadius(radius);
        }
    }

    void drawCylinder(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.03f, float length = 0.5f){
        if(!findEntity(name)){
            Qt3DCore::QEntity *parent;
            if(!getEntity(entity, parent)){
                std::cout << "COULD NOT FIND PARENT ENTITY "<< entity.toStdString()<<" TO DRAW CYLINDER "<< name.toStdString() << std::endl;
                return;
            }

            // object does not exist yet
            Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
            material->setDiffuse(c);
            material->setEnabled(visible);
            material_map_.insert(std::make_pair(name.toStdString(),material));

            Qt3DCore::QTransform* tf = new Qt3DCore::QTransform();
            QMatrix4x4 correction;
            correction.rotate(90.0f, QVector3D(1,0,0));
            correction.translate(QVector3D(0.0f, length/2 ,0.f));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf->setMatrix(pose_qmat*correction);
            tf_map_.insert(std::make_pair(name.toStdString(),tf));

            Qt3DExtras::QCylinderMesh* mesh = new Qt3DExtras::QCylinderMesh();
            mesh->setLength(length);
            mesh->setRadius(radius);
            cylinder_mesh_map_.insert(std::make_pair(name.toStdString(),mesh));

            Qt3DCore::QEntity *cylinderEntity = new Qt3DCore::QEntity(parent);
            cylinderEntity->addComponent(mesh);
            cylinderEntity->addComponent(tf);
            cylinderEntity->addComponent(material);
            entity_map_.insert(std::make_pair(name.toStdString(),cylinderEntity));
        }else{
            // object exists
            QMatrix4x4 correction;
            correction.rotate(90.0f, QVector3D(1,0,0));
            correction.translate(QVector3D(0.0f, length/2 ,0.f));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf_map_[name.toStdString()]->setMatrix(pose_qmat*correction);
            material_map_[name.toStdString()]->setDiffuse(c);
            material_map_[name.toStdString()]->setEnabled(visible);
            cylinder_mesh_map_[name.toStdString()]->setRadius(radius);
            cylinder_mesh_map_[name.toStdString()]->setLength(length);
        }
    }

    void drawCone(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = 0.16f, float length = 0.5f){
        if(!findEntity(name)){
            Qt3DCore::QEntity *parent;
            if(!getEntity(entity, parent)){
                std::cout << "COULD NOT FIND PARENT ENTITY "<< entity.toStdString()<<" TO DRAW CONE "<< name.toStdString() << std::endl;
                return;
            }

            // object does not exist yet
            Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
            material->setDiffuse(c);
            material->setEnabled(visible);
            material_map_.insert(std::make_pair(name.toStdString(),material));

            Qt3DCore::QTransform* tf = new Qt3DCore::QTransform();
            QMatrix4x4 correction;
            correction.rotate(90.0f, QVector3D(1,0,0));
            correction.translate(QVector3D(0.0f, length/2 ,0.f));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf->setMatrix(pose_qmat*correction);
            tf_map_.insert(std::make_pair(name.toStdString(),tf));

            Qt3DExtras::QConeMesh* mesh = new Qt3DExtras::QConeMesh();
            mesh->setLength(length);
            mesh->setBottomRadius(radius);
            mesh->setTopRadius(0.0f);
            mesh->setHasBottomEndcap(true);
            mesh->setHasTopEndcap(true);
            cone_mesh_map_.insert(std::make_pair(name.toStdString(),mesh));

            Qt3DCore::QEntity *cylinderEntity = new Qt3DCore::QEntity(parent);
            cylinderEntity->addComponent(mesh);
            cylinderEntity->addComponent(tf);
            cylinderEntity->addComponent(material);
            entity_map_.insert(std::make_pair(name.toStdString(),cylinderEntity));
        }else{
            // object exists
            QMatrix4x4 correction;
            correction.rotate(90.0f, QVector3D(1,0,0));
            correction.translate(QVector3D(0.0f, length/2 ,0.f));
            QMatrix4x4 pose_qmat = eigenToQt(pose);
            tf_map_[name.toStdString()]->setMatrix(pose_qmat*correction);
            material_map_[name.toStdString()]->setDiffuse(c);
            material_map_[name.toStdString()]->setEnabled(visible);
            cone_mesh_map_[name.toStdString()]->setBottomRadius(radius);
            cone_mesh_map_[name.toStdString()]->setLength(length);
        }
    }

    void drawArrow(QString entity, QString name, const Eigen::Matrix4f& pose, QColor c = QColor("blue"), bool visible = true, float radius = ARROW_RADIUS, float length = ARROW_LENGTH){
        drawCylinder(entity, name, pose, c, visible, radius, length);

        Eigen::Matrix4f move = Eigen::Matrix4f::Identity();
        move.block<3,1>(0,3) = Eigen::Vector3f(0,0,length);
        drawCone(entity, name+"_cone", pose*move, c, visible, radius*2, length/15);
    }

    void draw3DAxes(QString entity, QString name, Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(), bool visible = true, float scale = 1.0f){
        Eigen::Matrix4f z_to_x = Eigen::Matrix4f::Identity();
        z_to_x.block<3,3>(0,0) = static_cast<Eigen::Matrix3f>(Eigen::AngleAxisf(PI/2.0f, Eigen::Vector3f::UnitY()));
        drawArrow(entity, name+"_x_axis",pose*z_to_x, QColor("red"), visible, ARROW_RADIUS*scale, ARROW_LENGTH*scale);

        Eigen::Matrix4f z_to_y = Eigen::Matrix4f::Identity();
        z_to_y.block<3,3>(0,0) = static_cast<Eigen::Matrix3f>(Eigen::AngleAxisf(-PI/2.0f, Eigen::Vector3f::UnitX()));
        drawArrow(entity, name+"_y_axis",pose*z_to_y, QColor("green"), visible, ARROW_RADIUS*scale, ARROW_LENGTH*scale);

        drawArrow(entity, name+"_z_axis",pose, QColor("blue"), visible, ARROW_RADIUS*scale, ARROW_LENGTH*scale);

        drawSphere(entity, name, pose, QColor("yellow"), visible, 0.04f*scale);
    }

    void drawVector(QString entity, QString name, Eigen::Vector3f position, Eigen::Vector3f value, QColor c = QColor("blue"), bool visible = true, float radius = ARROW_RADIUS){

        Eigen::Vector3f v = Eigen::Vector3f(0,0,1).cross(value.normalized());
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

        if (!v.isZero()){
            float sin = v.norm();
            float cos = Eigen::Vector3f(0,0,1).dot(value.normalized());

            Eigen::Matrix3f skew;
            skew << 0 , -v(2), v(1),
                    v(2), 0, -v(0),
                    -v(1), v(0), 0;

            if (sin!=0)
                R = Eigen::Matrix3f::Identity() + skew+ skew*skew *(1-cos)/(sin*sin);
            else
                R = Eigen::Matrix3f::Identity() + skew;
        }
        else{
            if(value(2)>0)
                R.diagonal() << 1,1,1;
            else
                R.diagonal() << 1,-1,-1;
        }

        float length = value.norm();
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,1>(0,3) = position;
        pose.block<3,3>(0,0) = R;
        drawArrow(entity, name, pose, c, visible, radius, length);
    }

private:
    // Spheres map
    std::map<std::string, Qt3DExtras::QSphereMesh*> sphere_mesh_map_;
    // Cylinders map
    std::map<std::string, Qt3DExtras::QCylinderMesh*> cylinder_mesh_map_;
    // Cones map
    std::map<std::string, Qt3DExtras::QConeMesh*> cone_mesh_map_;
    // Planes map
    std::map<std::string, Qt3DExtras::QPlaneMesh*> plane_mesh_map_;
    // Boxs map
    std::map<std::string, Qt3DExtras::QCuboidMesh*> box_mesh_map_;
    // Entities map
    std::map<std::string, Qt3DCore::QEntity*> entity_map_;
    // Transformations map
    std::map<std::string, Qt3DCore::QTransform*> tf_map_;
    // Materials map
    std::map<std::string, Qt3DExtras::QPhongMaterial*> material_map_;
};

#endif // SCENEMODIFIER_H
