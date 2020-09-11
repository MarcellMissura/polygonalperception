#ifndef ROBOTCONTROLLOOP_H_
#define ROBOTCONTROLLOOP_H_

#include "util/StopWatch.h"
#include "util/Timer.h"
#include "RobotControl.h"

class RobotControlLoop : public QObject
{
    Q_OBJECT

    bool running;
    StopWatch stopWatch; // for precise performance measuring
    Timer timer; // High precicision timer that drives the rc thread
    double lastUpdateTimestamp;
    double lastStartTimestamp;

    RobotControl robotControl;

public:
    RobotControlLoop(QObject *parent = 0);
    ~RobotControlLoop(){}

    void init();
    void start();
    void stop();
    void smallStep(int frameIndex);
    void reset();

public slots:
    void step();
};

#endif
