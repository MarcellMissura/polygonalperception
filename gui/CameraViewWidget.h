#ifndef CAMERAVIEWWIDGET_H_
#define CAMERAVIEWWIDGET_H_

#include <QWidget>
#include <QImage>

class CameraViewWidget : public QWidget
{
    Q_OBJECT

    QImage image;

public:
    bool showPolygons;
    bool showFloorDetection;

public:
    CameraViewWidget(QWidget *parent = 0);
    ~CameraViewWidget(){}

public slots:
    void init();
    void frameIndexChangedIn(int cfi);
    void togglePolygons();
    void toggleFloorDetection();

protected:
    void paintEvent(QPaintEvent* paintEvent);
};

#endif // CAMERAVIEWWIDGET_H_
