#ifndef STATE_H_
#define STATE_H_

#include <QDebug>
#include <QList>
#include <QStringList>
#include <QMutex>
#include <typeinfo>
#include "util/Vec3.h"
#include "util/Transform3D.h"
#include "util/ColorUtil.h"
#include "GridModel.h"
#include "SampleGrid.h"

// Represents the current state of the robot and its perception of the world.
struct State
{
    int frameId;
    double time; // Current robot control time since program start.
    double debug; // An all purpose debug value.
    double realTime; // Current real time since program start.
    double rcIterationTime; // How long did the last RC iteration really take?
    double rcExecutionTime; // The execution time of the last RC iteration.
    double avgExecutionTime; // Running average of the execution time.

    GridModel gridModel;
    SampleGrid sampleGrid;
    Transform3D cameraTransform;
    Vector<Polygon> polygons;
    Sample floor;
    double numPolygons;
    double numVertices;

    Vec3 pointBuffer[NUMBER_OF_POINTS];
    Pixel colorBuffer[NUMBER_OF_POINTS];

    static QMutex gMutex;

 public:

    State();
    ~State(){}
    void init();
    void clear();
    void bufferAppend(int maxLength = 0);
    void bufferOverwrite(int frameIndex);
    void restore(int frameIndex);
    void bufferToFile();
    void saveHistory() const;
    void loadHistory(int maxLength);
    int size() const;
    State& operator[](int i);
    double operator()(int i) const;
    double operator()(QString key) const;
    double getMember(int i) const;
    double getMember(QString key) const;
    void setMember(int i, double v);
    void setMember(QString key, double v);

    static QStringList memberNames; // Contains the names of the members in the right order.

private:

    // Registers a member variable for index based access.
    template <typename T>
    void registerMember(QString name, T* member)
    {
        memberNames << name;
        memberOffsets << (quint64)member - (quint64)this;
        memberTypes << QString(typeid(*member).name());
        //qDebug() << name << memberTypes.last();
    }

    // These members are static so that buffering into history does not create copies.
    static QList<quint64> memberOffsets;
    static QList<QString> memberTypes;
    static QMutex mutex;
    static QList<State> history;
};

extern State state;

#endif /* STATE_H_ */

