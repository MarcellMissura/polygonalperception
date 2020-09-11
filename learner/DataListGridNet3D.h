#ifndef DATALISTGRIDNET3D_H_
#define DATALISTGRIDNET3D_H_
#include "DataListGridNet.h"
#include <QGLViewer/frame.h>

// The DataListGridNet3D extends the DataListGridNet with a 3D transformation capability.

class DataListGridNet3D : public DataListGridNet<3>
{
protected:

    static const uint DIM = 3;
    qglviewer::Frame transform; // Transform of the grid with respect to the world frame.

public:

    DataListGridNet3D();
    ~DataListGridNet3D(){}

    void reset();

    // Transform handling.
    void setTransform(double r, double p, double y);
    void setTransform(qglviewer::Frame tr);
    qglviewer::Frame getTransform();

    void load(QString name = "DataGridNet3D.dat");
    void save(QString name = "DataGridNet3D.dat");

    // Grid node interface.
    NVec<3> getNodeCoordinates(const QVector<uint> idx); // Returns the Grid coordinates of the node specified by the DIM dimensional index.
    NVec<3> getNodeCoordinates(uint idx); // Returns the Grid coordinates of the node specified by the flat index.
    QVector<uint> getNodeIndex(const NVec<3> &x); // Finds the index of the point x.
    QVector<uint> getNodeIndexBl(const NVec<3> &x); // Finds the bottom left index of the point x.
    uint getNodeFlatIndex(const NVec<3> &x); // Finds the flat index of the point x.
    uint getNodeFlatIndexBl(const NVec<3> &x); // Finds the bottom left flat index of the point x.
    bool containsPoint(const NVec<3> x); // Decides if the point is inside the grid boundaries.
    NVec<3> samplePoint(); // Returns a uniformly sampeled point from the grid space.

    // Data point interface.
    uint addDataPoint(const NVec<3> x, const double y); // Sorts a new point into the bin structure.
    QList< NVec<4> > getDataPoints(const QVector<uint> idx, uint r=0);
    QList< NVec<4> > getDataPoints(uint idx, uint r=0);
    QList< NVec<4> > getAllDataPoints();

    // Query interface.
    double evaluateAt(NVec<3> x);
    double evaluateAt(NVec<3> x, double& c);
};

#endif /* DATALISTGRIDNET3D_H_ */
