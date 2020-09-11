#ifndef POLYGONALPERCEPTION_H
#define POLYGONALPERCEPTION_H

#include "ui_polygonalperception.h"
#include <QMainWindow>
#include <QSplitter>
#include "gui/GraphWidget.h"
#include "gui/ConfigWidget.h"
#include "gui/CheckBoxWidget.h"
#include "gui/CameraViewWidget.h"
#include "gui/OpenGLWidget.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "RobotControlLoop.h"

class PolygonalPerception : public QMainWindow
{
    Q_OBJECT

    Ui::PolygonalPerceptionClass ui;

    CheckBoxWidget checkboxWidget;
    GraphWidget graphWidget;
    ConfigWidget configWidget;
    CameraViewWidget cameraViewWidget;
    OpenGLWidget openGLWidget;

    QSplitter* verticalSplitterTop;
    QSplitter* verticalSplitterBottom;
    QSplitter* horizontalSplitter;

    QAction* recordAction;

    int cfi;
    int tscale;
    bool recording;
    QTimer animationTimer;

    RobotControlLoop robotControlLoop;

public:
    PolygonalPerception(QWidget *parent = 0);
    ~PolygonalPerception();

public slots:

    void configChanged();
    void topSplitterMoved();
    void bottomSplitterMoved();
    void messageIn(QString m);
    void toggleConfig();
    void toggleGraph();
    void animate();
    void record();
    void play();
    void stop();
    void frameBack();
    void frameForward();
    void jumpToStart();
    void jumpToEnd();
    void jumpToFrame(int);

    void reset();
    void saveConfig();
    void loadConfig();
    void saveStateHistory();
    void clearStateHistory();
    void loadStateHistory();
    void loadFrame(int fi);
    void toggleFileBuffering();

signals:
    void frameIndexChangedOut(int);
    void progressOut(int);

protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    bool eventFilter(QObject *obj, QEvent *event);
};

#endif // NaoLab_H
