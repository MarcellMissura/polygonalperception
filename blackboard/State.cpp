#include <QMutexLocker>
#include <QDir>
#include <QDateTime>
#include <QDataStream>
#include "State.h"
#include "globals.h"
#include "util/ColorUtil.h"
#include "blackboard/Config.h"

/*
 * The State is a globally accessible, bufferable object with built in introspection
 * capabilities.
 *
 * The primary function of the state is to provide a central and easily accessible place
 * for the storage of system variables and objects, such as sensor values, actuator values,
 * and debug values. The state is globally accessible. This makes it easy to provide read
 * and write access to system variables to every component of the robot control software.
 * Anywhere in the code, just write for example state.time to access the current time.
 * All relevant state members are public.
 *
 * The state object is not thread safe. The reason why this still works is due to the
 * following. No state member is written from two different sources. Typically the robot
 * interface writes sensor values and the robot control reads them, or the robot control
 * writes actuator values and the robot interface reads them. The GUI only ever reads state
 * variables. Therefore, it cannot happen that two threads try to concurrently write the
 * same state member, where there result would be undefined. In addition, read and write
 * operations on up to 8 aligned bytes (such as doubles) are atomic. That means that a half
 * way written double does not exist. It is possible, however, that structures such as vectors
 * are partially written by one thread, while another thread is reading it. In this case it
 * can happen that for example that x and y components of a three dimensional vector are not
 * in sync with the z component. A situation where this is an actual problem has yet to be
 * found.
 *
 * The state object is bufferable. This means that upon calling state.buffer(), the state
 * object will add a copy of itself into an internal history with bounded length. The state
 * object is buffered in each cycle. To access a historical state object use for example
 * state[1].supportLegSign, which would tell you which leg the robot was standing on one
 * buffered state before the current one. The few private members the state object needs
 * for keeping the history and to implement the introspection features are static so that
 * they don't get copied when the state is buffered into history.
 *
 * The state object has introspection capabilities. For a generic way of accessing state variables,
 * e.g. listing all state members in a for loop at runtime, a list of the state members needs to be
 * provided at compile time. (Unlike Java, C++ does not have a native introspection interface.) Thus,
 * the introspection feature needs to be supported manually by register()-ing each member in the init()
 * function. What you get for the this extra effort is an index and key based based read access to the
 * state members, such as state(0) or state("leftHip.y") along with a meta structure called memberNames.
 * The introspection based access methods are mainly used by the GUI to automatically build the graph
 * selector check boxes in the bottom left, and the state member graphs on the bottom right, without
 * having to change the GUI code each time the state object changes. The introspection interface and
 * the state history can be combined. For example, state[11]("fusedAngle.x") gives you the lateral
 * fused angle 11 cycles ago.
 */



// These members are static so that buffering into history does not create copies.
QMutex State::mutex;
QStringList State::memberNames;
QList<quint64> State::memberOffsets;
QList<QString> State::memberTypes;
QList<State> State::history;
QMutex State::gMutex;

// Ugly hack that makes sure ColorUtil is constructed before State.
ColorUtil colorUtil;

State state;

// In the constructor members should be initialized where needed.
State::State()
{    
    debug = 0;
    frameId = 0;
    time = 0;
    realTime = 0;

    rcIterationTime = 0;
    rcExecutionTime = 0;
    avgExecutionTime = 0;

    numPolygons = 0;
    numVertices = 0;
}

// The init() method should be called after construction of the state object.
// Here, all state object members are registered to build a descriptor meta
// structure that allows index and key based access to the member values.
// If you don't want to see a certain state member on the gui, there is no
// need to register it.
void State::init()
{
    registerMember("frameId", &frameId);
    registerMember("time", &time);
    registerMember("debug", &debug);

    registerMember("timing.rcIterationTime", &rcIterationTime);
    registerMember("timing.rcExecutionTime", &rcExecutionTime);
    registerMember("timing.avgExecutionTime", &avgExecutionTime);

    registerMember("polygons", &numPolygons);
    registerMember("vertices", &numVertices);
}

// Clears the state history.
void State::clear()
{
	QMutexLocker locker(&mutex);
	history.clear();
    frameId = 0;
    time = 0;
}

// Saves the entire state history to a file.
void State::saveHistory() const
{
	QMutexLocker locker(&mutex);

    QFile file("data/statehistory.dat");
	file.open(QIODevice::WriteOnly);
	QDataStream out(&file);
    for (int i = history.size()-1; i >= 0; i--)
    {
        out << history[i].frameId;
        out << history[i].time;
        for (int j = 0; j < NUMBER_OF_POINTS; j++)
        {
            out << history[i].pointBuffer[j];
            out << history[i].colorBuffer[j].r;
            out << history[i].colorBuffer[j].g;
            out << history[i].colorBuffer[j].b;
        }
    }
	file.close();
}

// Loads the data file into the state history.
void State::loadHistory(int maxLength)
{
    QMutexLocker locker(&mutex);

    QFile file("data/statehistory.dat");
    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);

    //clear(); // can't call directly because mutex
    history.clear();

    int loadedFrames = 0;
    while(!in.atEnd() && loadedFrames < maxLength)
    {
        in >> frameId;
        in >> time;
        for (int i = 0; i < NUMBER_OF_POINTS; i++)
        {
            in >> pointBuffer[i];
            in >> colorBuffer[i].r;
            in >> colorBuffer[i].g;
            in >> colorBuffer[i].b;
        }

        //bufferAppend(maxLength); // can't call directly because mutex
        history.push_front(*this);
        loadedFrames++;
    }
    file.close();

    // Rewrite the frame numbers and times.
    for (int i = 0; i < history.size(); i++)
    {
        history[i].frameId = history.size()-i;
        history[i].time = history[i].frameId*config.rcIterationTime;
    }

    restore(0);
}

// Restores the state with the frameIndex from history into the current state.
void State::restore(int frameIndex)
{
    frameId = history[frameIndex].frameId;
    time = history[frameIndex].time;
    for (int i = 0; i < NUMBER_OF_POINTS; i++)
    {
        pointBuffer[i] = history[frameIndex].pointBuffer[i];
        colorBuffer[i] = history[frameIndex].colorBuffer[i];
    }
}

// Appends the current state to the state history.
void State::bufferAppend(int maxLength)
{
    QMutexLocker locker(&mutex);
    history.push_front(*this);
    while (maxLength > 0 && history.size() > maxLength)
        history.pop_back();
}

// Overwrites a state in the history with the current state.
void State::bufferOverwrite(int frameIndex)
{
    history[frameIndex] = *this;
}

void State::bufferToFile()
{
    QMutexLocker locker(&mutex);

    QFile file("data/statehistory.dat");
    file.open(QFile::Append);
    QDataStream out(&file);
    out << frameId;
    out << time;
    for (int j = 0; j < NUMBER_OF_POINTS; j++)
    {
        out << pointBuffer[j];
        out << colorBuffer[j].r;
        out << colorBuffer[j].g;
        out << colorBuffer[j].b;
    }

    file.close();
}

// Returns the amount of buffered historical state objects.
int State::size() const
{
	QMutexLocker locker(&mutex);
	return history.size();
}

// Returns a historical state object.
// i = 0 returns the current state.
// i = -1 (or 1) returns the state object from the iteration before and so on.
// To get the oldest known state you must request i = (+/-)state.size().
// Yes, unlike usual arrays, this operator handles items indices from 0 to state.size()!
// If abs(i) > state.size() the oldest known state object is returned.
State& State::operator[](int i)
{
	QMutexLocker locker(&mutex);

    if (i == 0 || history.empty())
		return *this;

    i = qMin(qAbs(i), history.size()) - 1;
	return history[i];
}

// Returns the value of the ith member of this object.
double State::operator()(int i) const
{
	return getMember(i);
}

// Returns the value of the member that was registered with the given key.
// If no member was registered with this key, 0 is returned.
// Index based access is faster than key based access.
double State::operator()(QString key) const
{
	return getMember(key);
}

// Returns the value of the ith member of this object.
double State::getMember(int i) const
{
	if (memberTypes[i].startsWith('i'))
        return (double)(*((int*)((quint64)this+memberOffsets[i])));
    if (memberTypes[i].startsWith('j'))
        return (double)(*((unsigned int*)((quint64)this+memberOffsets[i])));
	else if (memberTypes[i].startsWith('b'))
        return (double)(*((bool*)((quint64)this+memberOffsets[i])));
	else if (memberTypes[i].startsWith('f'))
        return (double)(*((float*)((quint64)this+memberOffsets[i])));
    return *((double*)((quint64)this+memberOffsets[i]));
}

// Returns the value of the member that was registered with the given key.
// If no member was registered with this key, 0 is returned.
// Index based access is faster than key based access.
double State::getMember(QString key) const
{
	int memberIndex = 0;
	bool memberFound = false;
    while (!memberFound)
		if (memberNames[memberIndex++] == key)
			memberFound = true;

	if (memberFound)
		return getMember(memberIndex-1);

	return 0;
}

// Sets the ith member to value v.
void State::setMember(int i, double v)
{
//	QMutexLocker locker(&mutex);

	if (memberTypes[i].startsWith('i'))
      *((int*)((quint64)this+memberOffsets[i])) = (int)v;
	else if (memberTypes[i].startsWith('b'))
        *((bool*)((quint64)this+memberOffsets[i])) = (bool)v;
	else if (memberTypes[i].startsWith('f'))
        *((float*)((quint64)this+memberOffsets[i])) = (float)v;
	else
        *((double*)((quint64)this+memberOffsets[i])) = (double)v;
}

// Sets the value of the member that was registered with the given key.
// If no member was registered with this key, nothing happens.
// Index based access is faster than key based access.
void State::setMember(QString key, double v)
{
	int memberIndex = 0;
	bool memberFound = false;
    while (!memberFound)
		if (memberNames[memberIndex++] == key)
			memberFound = true;

	if (memberFound)
		setMember(memberIndex-1, v);
	else
        qDebug() << "Unable to find state member" << key;
}
