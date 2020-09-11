#include "GPRDataGridNet3D.h"
#include <QGLViewer/vec.h>
using namespace qglviewer;

GPRDataGridNet3D::GPRDataGridNet3D()
{

}

// Performs a local neighborhood based training step.
void GPRDataGridNet3D::train()
{
    GPR<3> gpr;
    gpr.setMinMax(this->min, this->max);
    gpr.p1 = config.gprP1;
    gpr.p2 = config.gprP2; // kernel width
    int r = config.gprR;

    // For every node in the grid...
    for (uint n = 0; n < this->nodeCount; n++)
    {
        // Collect the data in the neighbourhood of the node.
        QVector<uint> nn = this->enumerateNeighborHood(n, r);
        QList< NVec<DIM+1> > data;
        for (int i=0; i < nn.size(); i++)
            data << bin[nn[i]];

        // Add (train) the data points to the GRP.
        QList<NVec<DIM> > X;
        QList<double> y;
        for (int i = 0; i < data.size(); i++)
        {
            NVec<DIM> v;
            for (uint d = 0; d < DIM; d++)
                v[d] = data[i][d];
            X << v;
            y << data[i].last();
        }
        gpr.clear();
        gpr.addData(X, y);

        // Query the GPR at the node location.
        NVec<DIM> x = Grid<DIM>::getNodeCoordinates(n); // untransformed version
        double mean = 0;
        double conf = 0;
        gpr.evaluateAt(x, mean, conf);

        // Set the output value.
        setNodeOutput(n, mean, 1.1-conf);

        qDebug() << n << "of" << this->nodeCount << "data points" << data.size() << "radius" << r << "nn size" << nn.size() << "mean:" << mean << "conf:" << conf;
    }
}

// Computes the GPR using all data in the grid.
void GPRDataGridNet3D::trainAll()
{
    GPR<DIM> gpr;
    gpr.setMinMax(this->min, this->max);
    gpr.p1 = config.gprP1;
    gpr.p2 = config.gprP2; // kernel width

    // Collect all data from the grid.
    QList< NVec<DIM+1> > data;
    for (uint n = 0; n < this->nodeCount; n++)
        data << bin[n];
    qDebug() << data.size() << "data points identified for training.";

    // Add (train) the data points to the GRP.
    QList<NVec<DIM> > X;
    QList<double> y;
    for (int i = 0; i < data.size(); i++)
    {
        NVec<DIM> v;
        for (uint d = 0; d < DIM; d++)
            v[d] = data[i][d];
        X << v;
        y << data[i].last();
    }
    gpr.addData(X, y);

    // For every node in the grid...
    for (uint n = 0; n < this->nodeCount; n++)
    {
        // Query the GPR at the node location.
        NVec<DIM> x = Grid<DIM>::getNodeCoordinates(n);
        double mean = 0;
        double conf = 0;
        gpr.evaluateAt(x, mean, conf);

        // Set the output value.
        setNodeOutput(n, mean, 1.1-conf);

        if (n % 100 == 0)
            qDebug() << n << "of" << this->nodeCount << "mean:" << mean << "conf:" << conf;
    }
}


