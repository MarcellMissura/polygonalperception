#ifndef LANDSCAPEWIDGET_H
#define LANDSCAPEWIDGET_H

#include "MessageQueue.h"
#include <QGLViewer/qglviewer.h>
#include "blackboard/State.h"
#include "util/StopWatch.h"

using namespace qglviewer;

class OpenGLWidget: public QGLViewer
{
	Q_OBJECT

public:
	bool recording;

    bool showPointCloud;
    bool showDiscardedPoints;
    bool showCameraTransform;
    bool showFloorDetection;
    bool showPolygons;
    bool showOccupancyMap;
    bool showFloor;

private:
    double radius;
    bool inited;
    MessageQueue messageQueue;

public:
    OpenGLWidget(QWidget* parent=0);
    ~OpenGLWidget();

public slots:
	void messageIn(QString m);
    void reset();

    void togglePointCloud();
    void toggleDiscardedPoints();
    void togglePolygons();
    void toggleAxis();
    void toggleCameraTransform();
    void toggleOccupancyMap();
    void toggleFloor();
    void toggleFloorDetection();

protected:
    void init();
	void draw();

private:
    void drawPoints();
    void drawCameraTransform();
    void drawOccupancyMap();
    void drawFloor();
    void drawPolygons();
    void drawFloorDetection();
};

#endif
