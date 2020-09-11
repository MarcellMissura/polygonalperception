#ifndef GRAPHICSSCENE_H_
#define GRAPHICSSCENE_H_

#include <QtGui>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include "util/StopWatch.h"

class GraphicsScene : public QGraphicsScene
{
    Q_OBJECT

    QFont font;
    QFont smallFont;

    int currentStateIndex;
    int lastCurrentStateIndex;

    QList<QGraphicsSimpleTextItem*> labels;

public:
    bool showLabels;

public:
    GraphicsScene(QObject *parent = 0);
    ~GraphicsScene(){}

    void init();

public slots:
    void reset();
    void frameIndexChangedIn(int cfi);
    void toggleLabels();

signals:
	void stateChangedOut();
};

#endif // GRAPHICSSCENE_H_
