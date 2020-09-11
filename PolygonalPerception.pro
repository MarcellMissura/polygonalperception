include(blackboard/blackboard.pri)
include(gui/gui.pri)
include(util/util.pri)
include(geometry/geometry.pri)
include(learner/learner.pri)

TEMPLATE = app
TARGET = PolygonalPerception
QT += core \
    gui \
    xml \
    opengl \
    network
HEADERS += PolygonalPerception.h \
    RobotControlLoop.h \
    RobotControl.h \
    GridModel.h \
    globals.h \
    SampleGrid.h
SOURCES += PolygonalPerception.cpp \
    RobotControlLoop.cpp \
    RobotControl.cpp \
    GridModel.cpp \
    SampleGrid.cpp \
    main.cpp
FORMS += polygonalperception.ui
RESOURCES +=
CONFIG += console
CONFIG += warn_off
CONFIG += c++11
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
#QMAKE_CXXFLAGS_RELEASE += -mavx
#QMAKE_CXXFLAGS_RELEASE += -ffast-math

LIBS += -L/usr/lib -L/usr/local/lib
LIBS += -L/usr/include/GL -lGLEW -lglut -lGLU -lGL
LIBS += -L/usr/include/opencv2 -lopencv_imgproc -lopencv_core -lz -ltbb
LIBS += -L/usr/include/QGLViewer -lQGLViewer-qt5
#LIBS += -L/usr/include/QGLViewer -lQGLViewer
LIBS += -L/usr/include/armadillo_bits -larmadillo -lopenblas -llapack -lblas
