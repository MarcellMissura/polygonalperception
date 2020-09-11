#include "CameraViewWidget.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "globals.h"
#include "util/ColorUtil.h"

CameraViewWidget::CameraViewWidget(QWidget *parent)
    : QWidget(parent)
{
    showPolygons = false;
    showFloorDetection = false;

    setMinimumWidth(IMAGE_WIDTH);
    setMaximumWidth(IMAGE_WIDTH);
    setMaximumHeight(IMAGE_HEIGHT);
}

void CameraViewWidget::init()
{

}

void CameraViewWidget::frameIndexChangedIn(int cfi)
{
    // Construct a new QImage from the raw data buffer in the state.
    image = QImage((uchar*)state.colorBuffer, IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    update();
}

void CameraViewWidget::togglePolygons()
{
    showPolygons = !showPolygons;
    update();
}

void CameraViewWidget::toggleFloorDetection()
{
    showFloorDetection = !showFloorDetection;
    update();
}

void CameraViewWidget::paintEvent(QPaintEvent*)
{
    // Mutex against the robot control loop.
    QMutexLocker locker(&state.gMutex);

    // Instantiate a QPainter.
    QPainter painter(this);

    // Draw the camera image onto the widget.
    painter.drawImage(QPoint(), image);

    // Draw the floor detection visualization onto the camera image.
    if (showFloorDetection)
        state.sampleGrid.drawSamples(&painter);
}

