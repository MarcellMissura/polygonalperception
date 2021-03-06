#include "RobotControlLoop.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "util/Statistics.h"
#include <QDebug>

// The main control loop is the main thread of the architecture.
// It contains the actual robot control, that implements the classic agent function
// action = f(percept), and an interface to the physical robot. The interface is
// used to receive a percept from the robot, pass it on to the agent function and
// then to pass the action generated by the agent on to the physical robot. In this
// particular implementation, the physical robot is a bullet simulated humanoid model.
// A high precision windows multimedia timer drives the main thread by periodically
// calling the step method. Even if this class is not a QThread, due to the windows
// mm timer the step method is executed in a separate thread.

RobotControlLoop::RobotControlLoop(QObject *parent) : QObject(parent)
{
	// Connect the internal timer.
    connect(&timer, SIGNAL(timeout()), this, SLOT(step()), Qt::DirectConnection); // It must be a direct connection for it to work right!
	running = false;
    lastUpdateTimestamp = 0;
    lastStartTimestamp = 0;
}

// Initialization cascade after construction.
void RobotControlLoop::init()
{
    robotControl.init();
}

// Reset to an initial state.
void RobotControlLoop::reset()
{

}

// Starts the main control loop.
void RobotControlLoop::start()
{
	running = true;
    timer.start((int)(config.rcIterationTime*1000));
	lastStartTimestamp = stopWatch.programTime();
}

// Stops the main control loop.
void RobotControlLoop::stop()
{
	running = false;
	timer.stop();
}

// The main loop of the game. It's triggered by the timer.
void RobotControlLoop::step()
{
    // This is a mutex against the gui draw() to avoid thread issues.
    // Be aware that this does influence the iteration time, but not the execution time.
    QMutexLocker locker(&state.gMutex);

    stopWatch.start();

	// Measure how much real time passed since the last tick.
    state.time += config.rcIterationTime;
	state.realTime = stopWatch.time();
    state.rcIterationTime = state.realTime - lastUpdateTimestamp;
	lastUpdateTimestamp = state.realTime;

    // Step the robot control (sense, act loop).
    stopWatch.start();
    robotControl.sense();
    robotControl.act();
  
	state.frameId++;

    // Measure execution time.
    state.rcExecutionTime = stopWatch.elapsedTime();
    state.avgExecutionTime = (state.avgExecutionTime*state.frameId+state.rcExecutionTime)/(state.frameId+1);

    // Buffer the state into history.
    state.bufferAppend(config.bufferSize);

    // Buffer also into a file if requested.
    if(command.bufferToFile)
        state.bufferToFile();
}

// Executes a robot control step without buffering and measuring time.
// This is to recompute things for a loaded state.
void RobotControlLoop::smallStep(int frameIndex)
{
    robotControl.sense();
    robotControl.act();
    state.bufferOverwrite(frameIndex);
}
