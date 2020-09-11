#ifndef LLKR_H_
#define LLKR_H_
#include "util/NVec.h"
#include "util/NMat.h"
#include "DataPoint.h"
#include <QList>
#include <QMutex>
#include <QMutexLocker>
#include <ANN/ANN.h>

class LLKR
{

private:

    QMutex mutex;

    QVector<DataPoint> data;

    NMat<3,3> T;
    NMat<3,3> Tinv;
    QVector<NVec<3> > transformedX; // Transformed data points for kd search.
    ANNkd_tree* kdTreeX; // search structure
    ANNkd_tree* kdTreeZ; // search structure
    bool initialized;

public:

    LLKR();
    ~LLKR();

    void save(QString name);
    void load(QString name);

    void reset();
    void init();
    void sort();
    void computeAllKernels();
    void setTransform(NMat<3,3> T);
    int getLoadedPointCount();
    int getKnownKernelCount();
    bool isKernelKnown(uint id);
    DataPoint findDataPoint(NVec<3> z);
    void setKernelData(uint id, NMat<3,3> Jinv, NVec<3> error);
    void computeKernelData(uint id);
    void addDataPoint(DataPoint dp);
    void addDataPoint(const NVec<3> x, const NVec<3> z);

    const QList<DataPoint> metricSearchX(NVec<3> x, int k=1);
    const QList<DataPoint> linearMetricSearchX(NVec<3> x, int k=1);
    const QList<DataPoint> knnSearchX(NVec<3> x, int k=1, double epsilon=0);
    const QList<DataPoint> knnSearchZ(NVec<3> y, int k=1, double epsilon=0);
    const QList<DataPoint> radiusSearchX(NVec<3> x, double r, int k=0, double epsilon=0);
    const QList<DataPoint> radiusSearchZ(NVec<3> y, double r, int k=0, double epsilon=0);
    DataPoint& operator[](int i);

    // OpenGL drawing code.
    void drawX(uint sampleFactor=1, bool showData=true, bool showKernels=true);
    void drawZ(uint sampleFactor=1);

    NMat<3,3> computePrincipalTransform();
    NMat<3,3> computeIsoTransform(double t);

};

#endif /* LLKR_H_ */
