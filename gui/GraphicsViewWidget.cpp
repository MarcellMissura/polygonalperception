#include "GraphicsViewWidget.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/StateUtil.h"

GraphicsViewWidget::GraphicsViewWidget(QWidget *parent)
    : QGraphicsView(parent)
{
//	setMouseTracking(true);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate); // Prevents tear when dragging an item.
    setRenderHint(QPainter::Antialiasing, true);

    //translate(width()/2, height()/2);
    scale(100, -100);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setDragMode(QGraphicsView::ScrollHandDrag);

    showAxis = false;
    showGrid = false;
    showRuler = false;
    showHud = false;
    recording = false;
    tscale = 0;
    currentStateIndex = 0;

    mouseClickTimeStamp = 0;
    lastMouseUpdateTimeStamp = 0;
    mouseDown = false;

    swipeFadeOutTimer.setInterval(5);
    lastSwipeFadeOutTimeStamp = 0;
    connect(&swipeFadeOutTimer, SIGNAL(timeout()), this, SLOT(swipeFadeOut()));

    connect(&messageQueue, SIGNAL(updated()), this, SLOT(update()));

//	setCacheMode(QGraphicsView::CacheBackground);
}

GraphicsViewWidget::~GraphicsViewWidget()
{

}

void GraphicsViewWidget::init()
{
    reset();
}

void GraphicsViewWidget::reset()
{
    stopSwipeFadeOut();
    scale(10/transform().m11(), -10/transform().m22());
    centerOn(20, 5.5);
}

void GraphicsViewWidget::messageIn(QString m)
{
    messageQueue.messageIn(m);
    update();
}

void GraphicsViewWidget::configChangedIn()
{

}

void GraphicsViewWidget::frameIndexChangedIn(int cfi)
{
    currentStateIndex = cfi;
}

void GraphicsViewWidget::startSwipeFadeOut()
{
    swipeFadeOutVelocity = mouseVelocity;
    lastSwipeFadeOutTimeStamp = stopWatch.programTime();
    windowCenter = mapToScene(width()/2, height()/2);
    swipeFadeOutTimer.start();
}

void GraphicsViewWidget::stopSwipeFadeOut()
{
    swipeFadeOutVelocity *= 0;
    swipeFadeOutTimer.stop();
}

void GraphicsViewWidget::startRecording()
{
    recording = true;
    update();
}

void GraphicsViewWidget::stopRecording()
{
    recording = false;
    update();
}

void GraphicsViewWidget::toggleAxis()
{
    showAxis = !showAxis;
    update();
}

void GraphicsViewWidget::toggleGrid()
{
    showGrid = !showGrid;
    update();
}

void GraphicsViewWidget::toggleRuler()
{
    showRuler = !showRuler;
    if (showRuler)
    {
        setMouseTracking(true);
        setDragMode(QGraphicsView::NoDrag);
        setCursor(Qt::CrossCursor);
    }
    else
    {
        setMouseTracking(false);
        setDragMode(QGraphicsView::ScrollHandDrag);
    }
    update();
}

void GraphicsViewWidget::toggleHud()
{
    showHud = !showHud;
    update();
}

// Handles the swipe fade out (inertial motion after the mouse button was released).
void GraphicsViewWidget::swipeFadeOut()
{
    if (swipeFadeOutVelocity.manhattanLength() < 0.85)
    {
        stopSwipeFadeOut();
        return;
    }

    double elapsedTime = (stopWatch.programTime() - lastSwipeFadeOutTimeStamp);
    QPointF mappedSwipeFadeOutVelocity = QPointF(swipeFadeOutVelocity.x()/transform().m11(), swipeFadeOutVelocity.y()/transform().m22());
    windowCenter -= mappedSwipeFadeOutVelocity*elapsedTime;
    centerOn(windowCenter);
    swipeFadeOutVelocity *= qMax(0.0, 1.0 - 5.0*elapsedTime);
    lastSwipeFadeOutTimeStamp = stopWatch.programTime();

    update();
}


// Updates the mouse state (position and velocity).
void GraphicsViewWidget::updateMouse(QPoint mousePos)
{
    mouse = mousePos;

    double timeDiff = (stopWatch.time()-lastMouseUpdateTimeStamp);
    if (timeDiff > 0.3)
    {
        mouseVelocity *= 0;
        lastMouse = mouse;
        lastMouseUpdateTimeStamp = stopWatch.time();
    }
    else if (timeDiff >= 0.003)
    {
        QPointF measuredMouseVelocity = (mouse - lastMouse)/timeDiff;
        mouseVelocity = 0.5*mouseVelocity + 0.5*measuredMouseVelocity;
        lastMouse = mouse;
        lastMouseUpdateTimeStamp = stopWatch.time();
    }
}

// Message queue and the frame info overlays, handles the auto cam too.
void GraphicsViewWidget::drawForeground(QPainter* painter, const QRectF& rect)
{
    painter->resetTransform(); // The painter draws in scene coordinates by default. We want draw in view coordinates.

//    // Item detector.
//    QGraphicsScene* sc = scene();
//    for (int i = 0; i < width(); i+=5)
//    {
//        for (int j = 0; j < width(); j+=5)
//        {
//            int itemCount = sc->items(mapToScene(i,j)).size();
//            if (itemCount > 0)
//                painter->drawEllipse(i, j, itemCount, itemCount);
//        }
//    }

    // Show recording state.
    if (recording)
    {
        painter->setFont(QFont("Helvetica", 18));
        painter->setPen(QColor::fromRgbF(0.8,0.3,0.3));
        painter->drawText(QPoint(18, 30), "Recording...");
    }

    // Draw the message queue.
    painter->save();
    painter->translate(18, 60);
    messageQueue.draw(painter, 85, 85, 0);
    painter->restore();

    // No data guard.
//	if (state.size() == 0)
//		return;

    if (showRuler)
    {
        QPointF mappedMouse = mapToScene(mouse);
        painter->drawText(mouse + QPoint(10, -10), "[" + QString::number(mappedMouse.x()) + ", " + QString::number(mappedMouse.y()) + "]");
    }

    if (showHud)
    {
        painter->setFont(QFont("Helvetica", 14, QFont::Light));
        painter->setPen(QColor::fromRgbF(0.3,0.3,0.7));
        painter->drawText(QPoint(18, height()-10), "frame: " + QString().number(state[currentStateIndex].frameId) +
                    "  time: " + QString().number(state[currentStateIndex].time, 'f', 3) +
                    "  debug: " + QString().number(state[currentStateIndex].debug, 'f', 3));
        painter->drawText(QPoint(width()-52, height()-10), "x" + QString().number(tscale, 'f', 1));
    }
}

// The axes and the grid are drawn on the background.
void GraphicsViewWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
    painter->setRenderHint(QPainter::Antialiasing, false);
    QPen pen(QColor(100, 10, 10));
    pen.setWidth(2);
    pen.setCosmetic(true);
    painter->setPen(pen);

    // Debug draw of window center and scene center.
    if (false)
    {
        QPointF windowCenter = mapToScene(width()/2, height()/2);
        painter->drawEllipse(windowCenter, 0.03, 0.03);
        QPointF sceneCenter = scene()->itemsBoundingRect().center();
        painter->drawEllipse(sceneCenter, 0.03, 0.03);
    }

    // draw grid
    if (showGrid)
    {
        if (transform().m11() > 150)
            drawGrid(painter, rect, 0.05, QColor(150, 150, 255, 100), Qt::DotLine);
        if (transform().m11() > 80)
            drawGrid(painter, rect, 0.1, QColor(150, 150, 255, 100), Qt::DashLine);
        if (transform().m11() > 4)
            drawGrid(painter, rect, 0.5, QColor(150, 150, 255, 100));
    }

    // draw axes
    if (showAxis)
    {
        double xmargin = 30/transform().m11(); // pixel
        double ymargin = 30/transform().m22(); // pixel
        double x = qBound(rect.left() + xmargin, 0.0, rect.right() - xmargin);
        double y = qBound(rect.top() - ymargin, 0.0, rect.bottom() + ymargin);
        painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
    }

//	setRenderHint(QPainter::Antialiasing, true);
}

void GraphicsViewWidget::drawGrid(QPainter *painter, const QRectF &rect, double step, QColor color, Qt::PenStyle penStyle)
{
    painter->save();

    QPen pen;
    pen.setColor(color);
    pen.setStyle(penStyle);
    pen.setCosmetic(true);
    painter->setPen(pen);

    double x = qRound(rect.left() / step) * step;
    double y = qRound(rect.top() / step) * step;
    while (x < rect.right())
    {
        painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        x += step;
    }
    while (y < rect.bottom())
    {
        painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
        y += step;
    }

    painter->restore();
}

void GraphicsViewWidget::wheelEvent(QWheelEvent *event)
{
    if (event->delta() < 0)
        scale(0.83, 0.83);
    else
        scale(1.2, 1.2);
    update();
}

void GraphicsViewWidget::mousePressEvent(QMouseEvent *event)
{
//    QGraphicsScene* sc = scene();
//    qDebug() << "mouse click widget";
//    qDebug() << "There are" << items(event->pos()).size() << sc->items(mapToScene(event->pos())).size() << "items at";
//    qDebug() << "screen position" << event->pos();
//    qDebug() << "scene position" << mapToScene(event->pos());

    mouseClick = event->pos();
    mouseClickTimeStamp = stopWatch.programTime();
    mouseDown = (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton));
    stopSwipeFadeOut();
    QGraphicsView::mousePressEvent(event);
}

void GraphicsViewWidget::mouseMoveEvent(QMouseEvent *event)
{
//    qDebug() << "mouse move widget";
//    qDebug() << "There are" << items(event->pos()).size()
//             << "items at position" << mapToScene(event->pos());

    updateMouse(event->pos());
    mouseDown = (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton));
    QGraphicsView::mouseMoveEvent(event); // use the default drag implementation
    update();
}

void GraphicsViewWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::RightButton)
    {
        scale(300/transform().m11(), -300/transform().m22());
    }
    else if (event->buttons() & Qt::LeftButton)
    {
        centerOn(0,0);
    }

    update();
}

void GraphicsViewWidget::mouseReleaseEvent(QMouseEvent *event)
{
    // Start swipe fade out if the screen was dragged and released.
    if (mouse != mouseClick && event->button() == Qt::LeftButton && items(event->pos()).size() == 0)
    {
        updateMouse(event->pos());
        startSwipeFadeOut();
    }

    mouseDown = (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton));

    QGraphicsView::mouseReleaseEvent(event);
}

