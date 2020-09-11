#include "GraphicsScene.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/StateUtil.h"
#include "util/ColorUtil.h"


GraphicsScene::GraphicsScene(QObject *parent)
    : QGraphicsScene(parent)
{
    //setSceneRect(0, 0, 100, 64);

	currentStateIndex = 0;
	lastCurrentStateIndex = 0;
    showLabels = false;

    font.setFamily("Arial");
    font.setPointSize(2);

    smallFont.setFamily("Arial");
    smallFont.setPointSize(1);
}

void GraphicsScene::init()
{

}

// Resets whatever needs to be reset when the user presses the reset button.
void GraphicsScene::reset()
{
    clear();
    init();
}

void GraphicsScene::frameIndexChangedIn(int cfi)
{
	lastCurrentStateIndex = currentStateIndex;
    currentStateIndex = cfi;


}

void GraphicsScene::toggleLabels()
{
    showLabels = !showLabels;
    for (int i = 0; i < labels.size(); i++)
        labels[i]->setVisible(showLabels);
    update();
}

