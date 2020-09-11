#ifndef POCSCENE_H_
#define POCSCENE_H_

#include <QtGui>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>

class PocScene : public QGraphicsScene
{
    Q_OBJECT

public:

    bool showTranslation;

    int currentStateIndex;
    int lastCurrentStateIndex;

    QGraphicsRectItem* car;
    QGraphicsEllipseItem* lWheel;
    QGraphicsEllipseItem* rWheel;
    QGraphicsEllipseItem* com;
    QGraphicsRectItem* stick;
    QGraphicsEllipseItem* carHandle;

    QGraphicsEllipseItem* com1;
    QGraphicsRectItem* stick1;
    QGraphicsEllipseItem* com2;
    QGraphicsRectItem* stick2;
    QGraphicsEllipseItem* com3;
    QGraphicsRectItem* stick3;

public:
    PocScene(QObject *parent = 0);
    ~PocScene(){}

    void init();

public slots:
    void reset();
    void setTime(double t);
    void configChangedIn();

signals:
	void stateChangedOut();

};

#endif // POCSCENE_H_
