#include "StateUtil.h"
#include "Config.h"

StateUtil stateUtil;

StateUtil::StateUtil()
{
	stateIndexOffset = 0;
}


// Returns the floor state index for the given time t.
// This is needed to find the state history index for a real time t.
int StateUtil::findIndex(double t)
{
	int stateIndex = qBound(1, int( double(state.size()-1) - t/config.rcIterationTime + stateIndexOffset), state.size()-1);
	while (stateIndex < state.size()-1 && state[stateIndex].time > t)
	{
		stateIndex++;
		stateIndexOffset++;
	}
	while (stateIndex > 1 && state[stateIndex-1].time <= t)
	{
		stateIndex--;
		stateIndexOffset--;
	}

	return stateIndex;
}

// Finds the minimum of all state members over the currently loaded state history.
State StateUtil::minState()
{
	State min = state[0];
	for (int i = state.size(); i > 0; i--)
		for (int j = 0; j < state.memberNames.size(); j++)
			if (min.getMember(j) > state[i].getMember(j))
				min.setMember(j, state[i].getMember(j));
	return min;
}

// Finds the maximum of all state members over the currently loaded state history.
State StateUtil::maxState()
{
	State max = state[0];
	for (int i = state.size(); i > 0; i--)
		for (int j = 0; j < state.memberNames.size(); j++)
			if (max.getMember(j) < state[i].getMember(j))
				max.setMember(j, state[i].getMember(j));
	return max;
}

