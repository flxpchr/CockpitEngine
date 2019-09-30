#ifndef QT_EIGEN_CONVERSIONS_HPP
#define QT_EIGEN_CONVERSIONS_HPP

#include <Eigen/Dense>
#include <QVector3D>
#include <QMatrix4x4>
namespace conversions {
    static QMatrix4x4 toQMatrix4x4(const Eigen::Matrix4f &e){
        QMatrix4x4 mat;
        for(int i=0;i<4;i++){
            const float* col = e.col(i).data();
            const QVector4D vec(col[0],col[1],col[2],col[3]);
            mat.setColumn(i, vec);
        }
        if(mat.isAffine())
            return mat;
        else
            return QMatrix4x4();
    }

    static QVector3D toQVector3D(const Eigen::Vector3f &e){
        QVector3D vect;
        vect.setX(e(0));
        vect.setY(e(1));
        vect.setZ(e(2));
        return vect;
    }
}
#endif // QT_EIGEN_CONVERSIONS_HPP
