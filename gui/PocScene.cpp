#include "PocScene.h"
#include "globals.h"
#include "framework/Config.h"
#include "framework/State.h"
#include "framework/StateUtil.h"
#include "util/ColorUtil.h"

PocScene::PocScene(QObject *parent)
    : QGraphicsScene(parent)
{
	setSceneRect(-1000, -1000, 2000, 2000);

    showTranslation = true;

	currentStateIndex = 0;
	lastCurrentStateIndex = 0;

	double wheelRadius = 0.05;
	double carHeight = 0.15;
	double carWidth = 0.3;


    stick = addRect(-0.5*0.025, 0, 0.025, state.poc.L, colorUtil.pen, colorUtil.brushYellow);
    car = addRect(-0.5*carWidth, -0.5*carHeight, carWidth, carHeight, colorUtil.pen, colorUtil.brushLightGray);
    lWheel = addEllipse(-wheelRadius, -wheelRadius, 2.0*wheelRadius, 2.0*wheelRadius, colorUtil.pen, colorUtil.brushMagenta);
    rWheel = addEllipse(-wheelRadius, -wheelRadius, 2.0*wheelRadius, 2.0*wheelRadius, colorUtil.pen, colorUtil.brushMagenta);
    com = addEllipse(-0.5*0.12, -0.5*0.12, 0.12, 0.12, colorUtil.pen, colorUtil.brushRed);
    carHandle = addEllipse(0, 0, 0, 0, colorUtil.pen, colorUtil.brushRed);

	lWheel->setParentItem(car);
	lWheel->setPos(car->rect().x() + 0.5*lWheel->rect().width(), car->rect().y() - 0.5*lWheel->rect().height());

	rWheel->setParentItem(car);
	rWheel->setPos(car->rect().x() + car->rect().width() - 0.5*rWheel->rect().width(), car->rect().y() - 0.5*rWheel->rect().height());

	com->setPos(0, stick->rect().height());
	com->setParentItem(stick);
	stick->setParentItem(car);
	stick->setPos(0, 0.5*carHeight);

	car->setParentItem(carHandle);
	car->setPos(0, 2.0*wheelRadius+0.5*carHeight);


    stick1 = addRect(-0.5*0.025, 0, 0.025, state.poc.L, colorUtil.pen, colorUtil.brushLightGray);
    com1 = addEllipse(-0.5*0.12, -0.5*0.12, 0.12, 0.12, colorUtil.pen, colorUtil.brushLightGray);

	com1->setPos(0, stick1->rect().height());
	com1->setParentItem(stick1);
	stick1->setParentItem(car);
	stick1->setPos(0, 0.5*carHeight);
	stick1->hide();
	com1->hide();
}

void PocScene::init()
{
	reset();
}

// Resets whatever needs to be reset when the user presses the reset button.
void PocScene::reset()
{

}

void PocScene::setTime(double t)
{
	lastCurrentStateIndex = currentStateIndex;
	currentStateIndex = stateUtil.findIndex(t);

	if (state[currentStateIndex].poc.phi == state[currentStateIndex].poc.phi)
		stick->setRotation(state[currentStateIndex].poc.phi * -180.0/PI);
	if (showTranslation and state[currentStateIndex].poc.x == state[currentStateIndex].poc.x)
		carHandle->setPos(state[currentStateIndex].poc.x, 0);

	if (state[currentStateIndex].bestScoreNode.state.phi == state[currentStateIndex].bestScoreNode.state.phi)
		stick1->setRotation(state[currentStateIndex].bestScoreNode.state.phi * -180.0/PI);
}

void PocScene::configChangedIn()
{

}


