#include "RobotControl.h"
#include "blackboard/Command.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "globals.h"
#include "util/Statistics.h"
#include "util/StopWatch.h"

RobotControl::RobotControl(QObject *parent) : QObject(parent)
{

}

// Initialization cascade after construction.
void RobotControl::init()
{
    QMutexLocker locker(&state.gMutex);
    state.gridModel.init();
    state.sampleGrid.init();
}

// Processes the sensor input to a world model.
void RobotControl::sense()
{   
    // Run the floor detection.
    state.sampleGrid.update(); // Pulls samples from the point cloud in state.pointBuffer.
    state.floor = state.sampleGrid.findFloor();
    state.cameraTransform.setFromGroundPlane(state.floor.n, state.floor.p);

    // Sort all the points into an occupancy map.
    Vec3 p;
    state.gridModel.clear();
    for (uint i = 0; i < NUMBER_OF_POINTS; i++)
    {
        if (state.pointBuffer[i].isNull())
            continue;

        p = state.cameraTransform * state.pointBuffer[i];

        if (p.z < config.floor || p.z > config.ceiling)
            continue;

        if (!state.gridModel.containsPoint(p))
            continue;

        Vec2u idx = state.gridModel.getNodeIndex(p);
        state.gridModel.setAt(idx, 255);
    }

    // Dilate the occupancy map.
    state.gridModel.dilate(config.dilationRadius);
    state.gridModel.setBorder(0);

    // Extract the polygons from the occupancy map.
    // The polygons are written into state.polygons.
    state.gridModel.extractPolygons();
}

// Generates an action for the agent given the current state of the world, goals, and commands.
void RobotControl::act()
{

}
