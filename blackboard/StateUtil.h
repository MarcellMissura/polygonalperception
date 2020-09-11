#ifndef STATEUTIL_H_
#define STATEUTIL_H_

#include "blackboard/State.h"

class StateUtil
{
	int stateIndexOffset;

public:
	StateUtil();
    ~StateUtil(){}
	int findIndex(double t);
	State minState();
	State maxState();
};

extern StateUtil stateUtil;

#endif /* STATEUTIL_H_ */

