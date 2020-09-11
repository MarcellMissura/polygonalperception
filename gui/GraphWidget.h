#ifndef GRAPH_H
#define GRAPH_H
#include <QWidget>
#include "blackboard/State.h"
#include "Curve.h"
#include "util/StopWatch.h"
#include "util/Vec2.h"
#include "blackboard/StateUtil.h"
#include "util/ColorUtil.h"

class GraphWidget: public QWidget
{
	Q_OBJECT

public:

   	QList<int> idsOfTheCurvesToShow; // Using this list to loop through accelerates the paint method.
   	QList<Curve> curves; // A list of all curves.
    bool recording;

private:
   	double t; // A time offset. Which state.time value is at the 0 mark of the x axis?
   	bool showAxes;
   	QPointF screenScale;
	QPointF screenOffset;
	QTransform screenTransform;

	StopWatch stopWatch;
	ColorUtil colorUtil;

	bool mousePresent; // Is the mouse on the widget or not.
	QPointF mouse; // current mouse location in pixel coordinates
	QPointF mappedMouse; // current mouse location in logical coordinates
	QPointF lastMouse; // last mouse location in pixel coordinates
	QPointF mouseDiff; // last mouse motion in pixel coordinates
	QPointF mappedMouseDiff; // last mouse motion in logical coordinates
	QPointF mouseVelocity; // estimated mouse velocity in pixels per second
	QPointF mappedMouseVelocity; // estimated mouse velocity in logical units per second
	QPointF mouseClick; // mouse click location in pixel coordinates
	double mouseClickTimeStamp;
	double lastMouseUpdateTimeStamp;

	double mouseCatchRadius;
	QPointF swipeFadeOutVelocity;
	double lastSwipeFadeOutTimeStamp;
	bool dragMoved;
	int draggedCurveId;
	double draggedCurveX;
	int nearestCurveId;
	QTimer swipeFadeOutTimer;

public:
	GraphWidget(QWidget* parent=0);
	~GraphWidget();
	void init();
    Vec2 getLeftRightStateIndex();

public slots:
    void frameIndexChangedIn(int cfi);
	void setShowCurve(int id, bool on);
	void swipeFadeOut();
	void exportData();
	void resampleColors();

signals:
	void messageOut(QString m);

protected:
	void paintEvent(QPaintEvent*);
	void keyReleaseEvent(QKeyEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseDoubleClickEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void leaveEvent(QEvent *event);
	void resizeEvent(QResizeEvent *event);

private:
	void updateMouse(QPointF mousePos);
	void updateScreenTransform();
};

#endif // GRAPH_H
