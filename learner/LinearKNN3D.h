#ifndef LINEARKNN3D_H_
#define LINEARKNN3D_H_
#include "util/NVec.h"
#include "util/NMat.h"
#include <QList>

// This is a simple linear KNN retriever class. It is linear in two ways.
// 1: It can only perform a dumb linear search through all data.
// 2: A 3D linear transformation matrix can be set to define the metric for the search.
// The input and output dimensions are specified as template parameters.
// The input dimensions (DIMIN) span the data space where the actual KNN search is performed.
// The output dimensions(DIMOUT) are just additional values that are attached to each data point.
// Use addDataPoint() to feed the KNN with data.

class LinearKNN3D
{

private:

    int loadedPoints; // number of actually loaded points
    QVector<double> data;

    NVec<3> mean;
    NMat<3,3> T;
    NMat<3,3> Tinv;

public:

    LinearKNN3D();
    ~LinearKNN3D();

    void reset();

    int getLoadedPointCount();

    void addDataPoint(const NVec<6> x);

    NVec<3> getMean();
    void setMean(NVec<3> m);
    NMat<3,3> getTransform();
    NMat<3,3> computeCovarianceTransform();
    void setTransform(const NMat<3,3> T_);
    void transformPoints();

    double norm(NVec<3> x, int p=2);
    double norm(NVec<3> x, NVec<3> y, int p=2);

    const QList<NVec<6> > getDataPoints();

    const QList<NVec<7> > knnSearch(NVec<3> x, int k=1, int normP=2);
    const QList<NVec<7> > planeSelect(NVec<3> x, NVec<3> normal, int k=1);
    const QList<NVec<7> > lineSelect(NVec<3> x, NVec<3> v, int k=1);
    const QList<NVec<7> > angleSelect(NVec<3> x, NVec<3> v, int k=1);
    void angleDelete(NVec<3> x, NVec<3> v, double thresh=0.1);

    // OpenGL drawing code.
    void draw(uint sampleFactor=1, bool untransform=false, bool showData=true, bool showAxes=true);
};

#endif /* KNN_H_ */
