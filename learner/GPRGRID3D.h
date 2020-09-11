#ifndef GPRGRID3D_H_
#define GPRGRID3D_H_
#include "GRID3D.h"
#include <QHash>

class GPRGRID3D : public GRID3D
{
    int activationRadius; // Size of the neighbourhood that acts activated when adding new data.
    int maxBinSize; // The maximum number of points allowed to be in a bin. Older points are discarded.

    QVector< QList< NVec<DIM+1> > > bin;// Data bins assigned to the som neurons.
    QVector<double> centroid; // Centroids of the data bins assigned to the som neurons.
    QHash<int, bool> active; // Activation flag to speed up training.

public:

    GPRGRID3D();
    ~GPRGRID3D(){}

    // Parameter setters.
    void setActivationRadius(int ar);
    void setMaxBinSize(int maxBinSize);

    // Grid construction interface.
    void setN(int N_);
    void setN(const int* N_);

    // Grid population interface.
    void addDataPoint(NVec<3> x, double y);
    void addData(QList< NVec<3> > X, QList<double> y);
    void clearData();
    void printData();
    bool containsPoint(NVec<3> x);
    void eraseData(const int *idx);
    void eraseData(int idx);
    QList< NVec<4> > getData(const int *idx, int r=0);
    QList< NVec<4> > getData(int idx, int r=0);
    QList<double> getCentroids(const int *idx, int r=0);
    QList<double> getCentroids(int idx, int r=0);


    // Training interface.
    void setNodeOutputsToDataCentroids();
    void activate(int n);
    void train();
    void trainAllData();

    // OpenGL drawing.
    void drawGridSlice(double z=0, int sampleFactor=1, double zScale=1.0, double colorContrast=1.0, double transparency=0, bool showData=false);
    void drawData();
};


#endif /* SOM3D_H_ */
