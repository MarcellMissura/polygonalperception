#ifndef SOM3D_H_
#define SOM3D_H_
#include "GRID3D.h"
#include <QHash>

class Som3D : public GRID3D
{
    static const int DIM = 3;

    double dataGain; // How strongly the som neurons are pulled towards the data centroids.
    double neighborhoodGain; // How strongly the som neurons are pulled towards their neighborhood centroids.
    double timestep; // Simulation time step.
    int maxBinSize; // The maximum number of points allowed to be in a bin. Older points are discarded.
    int blurKernelWidth; // The width of the smoothing kernel.
    int activationRadius; // Size of the neighbourhood that acts activated when adding new data.

    QVector< QList< NVec<DIM+1> > > bin;// Data bins assigned to the som neurons.
    QVector<double> centroid; // Centroids of the data bins assigned to the som neurons.
    QVector<double> centroidForce; // The currently acting centroid force on each neuron.
    QVector<double> firstOrderForce; // The currently acting first order force on each neuron.
    QVector<double> filterBuffer; // Working buffer for filters.
    QHash<int, bool> active; // Activation flag to speed up training.

public:

    Som3D();
    ~Som3D(){}

    // Parameter setters.
    void setDataGain(double dataGain);
    void setNeighborhoodGain(double neighborhoodGain);
    void setTimestep(double timestep);
    void setMaxBinSize(int maxBinSize);
    void setBlurKernelWidth(int kw);
    void setActivationRadius(int ar);

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
    void train(int howManySteps=1);
    void blur();
    void setNodeOutputsToDataCentroids();
    void activate(int n);
    void gprTraining();

    // OpenGL drawing.
    void drawGridSlice(double z=0, int sampleFactor=1, double zScale=1.0, double colorContrast=1.0, bool showData=false);
    void drawData();

private:
    void oldTrainingStep();
    void trainingStep();

};


#endif /* SOM3D_H_ */
