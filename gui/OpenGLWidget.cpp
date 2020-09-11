#include "OpenGLWidget.h"
#include "blackboard/Config.h"
#include "blackboard/StateUtil.h"
#include "blackboard/Command.h"
#include <QKeyEvent>
#include <GL/glu.h>
#include "util/GLlib.h"
#include "util/ColorUtil.h"
#include "globals.h"

// The OpenGLWidget offers a 3D view where basically anything can be visualized.
// It's based on the QGLViewer library that offers great possibilities to create
// an OpenGL environment, move the camera in it with the mouse and it also comes
// with a library for 3D transformations.
OpenGLWidget::OpenGLWidget(QWidget *parent) : QGLViewer(parent)
{
    radius = 6.0;
    recording = false;
    showPointCloud = true;
    showDiscardedPoints = false;
    showPolygons = true;
    showOccupancyMap = true;
    showFloor = false;
    inited = false;
    showCameraTransform = true;
    showFloorDetection = false;

    connect(&messageQueue, SIGNAL(updated()), this, SLOT(update()));
}

void OpenGLWidget::reset()
{

}

void OpenGLWidget::init()
{
    restoreStateFromFile();

//	setBackgroundColor(QColor(245,245,245));
    setBackgroundColor(QColor(255,255,255));
	setForegroundColor(QColor(0,0,0));
	setFont(QFont("Helvetica", 18));

    setAxisIsDrawn(false);
	setSceneRadius(radius);

	// Make camera the default manipulated frame.
    //setManipulatedFrame(camera()->frame());

	// Light setup
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_LINE_SMOOTH);

    inited = true;
}

OpenGLWidget::~OpenGLWidget()
{
    if (inited)
        saveStateToFile();
}

void OpenGLWidget::messageIn(QString m)
{
	messageQueue.messageIn(m);
	update();
}

void OpenGLWidget::toggleAxis()
{
	toggleAxisIsDrawn();
	update();
}

void OpenGLWidget::togglePointCloud()
{
    showPointCloud = !showPointCloud;
    update();
}

void OpenGLWidget::toggleDiscardedPoints()
{
    showDiscardedPoints = !showDiscardedPoints;
    update();
}

void OpenGLWidget::toggleCameraTransform()
{
    showCameraTransform = !showCameraTransform;
    update();
}

void OpenGLWidget::togglePolygons()
{
    showPolygons = !showPolygons;
    update();
}

void OpenGLWidget::toggleOccupancyMap()
{
    showOccupancyMap = !showOccupancyMap;
    update();
}

void OpenGLWidget::toggleFloor()
{
    showFloor = !showFloor;
    update();
}

void OpenGLWidget::toggleFloorDetection()
{
    showFloorDetection = !showFloorDetection;
    update();
}

void OpenGLWidget::draw()
{
    // Mutex against the step of robot control loop.
    QMutexLocker locker(&state.gMutex);

    if (showFloor)
        drawFloor();

    if (showCameraTransform)
        drawCameraTransform();

    if (showPointCloud)
        drawPoints();

    if(showFloorDetection)
        drawFloorDetection();

    if (showOccupancyMap)
        drawOccupancyMap();

    if (showPolygons)
        drawPolygons();

	// Show recording state.
	if (recording)
	{
		glColor3f(0.8, 0.3, 0.3);
		drawText(10, 24, "Recording...", QFont("Helvetica", 18));
	}

	// On the top: show the message queue.
	for (int i = 0; i < messageQueue.messages.size(); i++)
	{
		glColor4f(0.3, 0.3, 0.0, messageQueue.fadeFactors[i]);
		drawText(10, 52 + i*28, messageQueue.messages[i], QFont("Helvetica", 18));
	}

    // On the bottom: show the frame id and debug information.
	glColor3f(0.3, 0.3, 0.8);
    drawText(10, this->height() - 10, "frame: " + QString().number(state.frameId) +
            "/" + QString().number(state.size()) +
             "  polygons: " + QString().number(state.numPolygons) +
             "  vertices: " + QString().number(state.numVertices),
			QFont("Helvetica", 14, QFont::Light));
}

// Draws the height map.
void OpenGLWidget::drawOccupancyMap()
{
    glPushMatrix();
    glTranslated(0, 0, config.heightmapDz);
    state.gridModel.draw();
    glPopMatrix();
}

// Draws the floor.
void OpenGLWidget::drawFloor()
{
    float size = 2.0*radius;
    float stride = 0.05;
    glBegin( GL_QUADS );
    glColor3f(0.95, 0.95, 0.95);
    glVertex3f(-size, -size, 0);
    glVertex3f(-size, size, 0);
    glVertex3f(size, size, 0);
    glVertex3f(size, -size, 0);
    glEnd();

    glLineWidth(1);
    glBegin( GL_LINES );
    glColor3f(0.65, 0.65, 0.65);
    for (float i = -size; i <= size + 0.0001; i = i+stride)
    {
        glVertex3f(i, -size, 0.001);
        glVertex3f(i, size, 0.001);
        glVertex3f(-size, i, 0.001);
        glVertex3f(size, i, 0.001);
    }
    glEnd();
}

// Draws the computed floor normals.
void OpenGLWidget::drawFloorDetection()
{
    glPushMatrix();
    glMultMatrixd(state.cameraTransform);
    glTranslated(0, 0, config.floorDz);

    // Sample floor normals.
    state.sampleGrid.drawSamples();

    // The final floor normal.
    if (true)
    {
        glPushMatrix();
        glTranslated(state.floor.p.x, state.floor.p.y, state.floor.p.z);
        glColor3f(0.0, 0.0, 1.0);
        QGLViewer::drawArrow(qglviewer::Vec(0,0,0), qglviewer::Vec(state.floor.n.normalized(0.5)), 0.01);
        glPopMatrix();
    }

    glPopMatrix();
}

// Draws the camera transform.
void OpenGLWidget::drawCameraTransform()
{
    glPushMatrix();
    glMultMatrixd(state.cameraTransform);
    QGLViewer::drawAxis(0.3);
    glPopMatrix();
}

// Draw the point buffer.
void OpenGLWidget::drawPoints()
{
    glPushMatrix();
    glMultMatrixd(state.cameraTransform);
    glPointSize(3);
    glBegin(GL_POINTS);

    for (int i = 0; i < NUMBER_OF_POINTS; i++)
    {
        if (state.pointBuffer[i].isNull())
            continue;

        if (!showDiscardedPoints)
        {
            Vec3 p = state.cameraTransform*state.pointBuffer[i];
            if (p.z < config.floor)
                continue;
        }

        glColor3ubv((GLubyte*)&state.colorBuffer[i]);
        glVertex3dv(state.pointBuffer[i]);
    }

    glEnd();
    glPopMatrix();
}

// Draws the polygons.
void OpenGLWidget::drawPolygons()
{
    glPushMatrix();
    glLineWidth(5);
    glTranslated(0, 0, config.polygonsDz);
    for (int i = 0; i < state.polygons.size(); i++)
    {
        //QColor c = colorUtil.sampleUniformColor();
        //glColor3f(c.redF(), c.greenF(), c.blueF());
        glColor3f(1, 0, 0);
        state.polygons[i].draw();
    }
    glPopMatrix();
}
