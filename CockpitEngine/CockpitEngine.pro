#-------------------------------------------------
#
# Project created by QtCreator 2017-07-20T11:47:05
#
#-------------------------------------------------

QT       += core charts gui network websockets serialport multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CockpitEngine
TEMPLATE = app


DEFINES += QT_DEPRECATED_WARNINGS

QVIRTUOSE_PATH = Q:/SmartLaparoProject/QVirtuoseRefactored
QSERVICEMANAGER_PATH = Q:/SmartLaparoProject/QServiceManager
QWINDOWSREGISTRY_PATH = Q:/SmartLaparoProject/QWindowsRegistry
VIRTUOSEAPI_PATH = Q:/lib/VirtuoseAPI_v3_97/win
EIGEN_PATH = Q:/lib/eigen
BOOST_PATH = Q:/lib/boost_1_66_0
#IPC_PATH = Q:/lib/IPC

#INCLUDEPATH += $${IPC_PATH}/Inc
INCLUDEPATH += $${QVIRTUOSE_PATH}/include
INCLUDEPATH += $${VIRTUOSEAPI_PATH}/include
INCLUDEPATH += $${EIGEN_PATH}
INCLUDEPATH += $${BOOST_PATH}
INCLUDEPATH += $${QSERVICEMANAGER_PATH}/include
INCLUDEPATH += $${QWINDOWSREGISTRY_PATH}/include
INCLUDEPATH += include

#Debug:LIBS += -L$${IPC_PATH}/x64/Debug
#Release:LIBS += -L$${IPC_PATH}/x64/Release
LIBS += -L$${QVIRTUOSE_PATH}/lib/x64
LIBS += -L$${VIRTUOSEAPI_PATH}/lib/VC2015/x64/Release
LIBS += -L$${QSERVICEMANAGER_PATH}/lib/x64
LIBS += -L$${QWINDOWSREGISTRY_PATH}/lib/x64
LIBS += -L$${BOOST_PATH}/stage/lib/

LIBS += -lAdvapi32 -lvirtuoseDLL -liphlpapi -lWs2_32 -lShell32# -lIPC -lIPC.Containers
Debug:LIBS      += -lqservicemanager-qt$${QT_VERSION}d -lqwindowsregistry-qt$${QT_VERSION}d -lqvirtuose-qt$${QT_VERSION}d
Release:LIBS    += -lqservicemanager-qt$${QT_VERSION} -lqwindowsregistry-qt$${QT_VERSION} -lqvirtuose-qt$${QT_VERSION}

QMAKE_LFLAGS += /NODEFAULTLIB:libcmt

SOURCES += src/main.cpp\
    src/trocar_detection.cpp \
    src/gravity_compensation.cpp \
    src/intelligent_switch.cpp \
    src/viscous_fields.cpp \
    src/wslogger.cpp \
    src/mainwindow.cpp \
    src/robot.cpp \
    src/robot_kinematics.cpp \
    src/camera_angle_controller.cpp \
    src/follow_tip_controller.cpp \
    src/joint_position_controller.cpp \
    src/wall_controller.cpp

HEADERS  += \
    include/trocar_detection.hpp \
    include/gravity_compensation.hpp \
    include/intelligent_switch.hpp \
    include/viscous_fields.hpp \
    include/filtering.hpp \
    include/eigen_posemat.hpp \
    include/wslogger.h \
    include/mainwindow.hpp \
    include/robot.hpp \
    include/instrument.hpp \
    include/robot_kinematics.hpp \
    include/constants.hpp \
    include/camera_angle_controller.hpp \
    include/smartledstripclient.h \
    include/simpleHttpServer/client_http.hpp \
    include/simpleHttpServer/server_http.hpp \
    include/follow_tip_controller.hpp \
    include/keypresseater.hpp \
    include/joint_position_controller.hpp \
    include/wall_controller.hpp \
    include/modes.hpp

FORMS    +=

#Release:QMAKE_POST_LINK += copy release\*.lib "$$BASEPATH\lib" $$escape_expand(\n\t)
#Debug:QMAKE_POST_LINK += copy debug\*.lib "$$BASEPATH\lib" $$escape_expand(\n\t)


