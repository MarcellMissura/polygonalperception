#include "GPRGRID3D.h"
#include "globals.h"
#include "GPR.h"
#include "GL/gl.h"
#include <util/ColorUtil.h>
#include "util/GLlib.h"
#include "framework/Config.h"
#include <QGLViewer/vec.h>
#include <QGLViewer/quaternion.h>
using namespace qglviewer;

GPRGRID3D::GPRGRID3D()
{
    activationRadius = 1;
    maxBinSize = 0;  // The maximum number of points allowed to be in a bin. Older points are discarded. 0 means no limit.
}

// Sets the radius of the neighbourhood that gets activated around each
// touched grid node when new data arrive.
void GPRGRID3D::setActivationRadius(int ar)
{
    activationRadius = ar;
}

// Sets the maximum number of allowed data points in a bin.
// A bin size of 0 means no limitation will be applied.
// Can be changed any time, even during training.
void GPRGRID3D::setMaxBinSize(int maxBinSize)
{
    this->maxBinSize = maxBinSize;
}

// Sets N, the number of nodes per dimension. This method sets N to be the same
// for every dimension and thus creates a uniform grid.
void GPRGRID3D::setN(int N_)
{
    GRID3D::setN(N_);

    int nodeCount = getNodeCount();
    bin.resize(nodeCount);
    centroid.resize(nodeCount);
}

// Sets N, the number of nodes per dimension. This method sets N[d] for each dimension individually.
// The size of the array is DIM, of course.
void GPRGRID3D::setN(const int* N_)
{
    GRID3D::setN(N_);

    int nodeCount = getNodeCount();
    bin.resize(nodeCount);
    centroid.resize(nodeCount);
}

// Prints a list of the data stored in the som.
void GPRGRID3D::printData()
{
    for (int i = 0; i < bin.size(); i++)
        for (int j = 0; j < bin[i].size(); j++)
            qDebug() << bin[i][j];
}

// Clears the data bins, but does not change the output values.
void GPRGRID3D::clearData()
{
    for (int i = 0; i < bin.size(); i++)
        bin[i].clear();
}

// Batch adds a bunch of points.
void GPRGRID3D::addData(QList< NVec<3> > X, QList<double> y)
{
    for (int i = 0; i < X.size(); i++)
        addDataPoint(X[i], y[i]);
}

// Adds a single data point with input values x and output value y to the data set.
void GPRGRID3D::addDataPoint(NVec<3> x, double y)
{
    // Discard NaN cases.
    if (x != x || y != y)
        return;

    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    x = NVec<3>(trx.x, trx.y, trx.z);

    // Discard out of range points.
    for (int d = 0; d < DIM; d++)
        if (x[d] > max[d] || x[d] < min[d])
            return;

    // Calculate the index of the cell that contains x.
    int idx[DIM];
    getNodeIndex(x, idx);
    for (int d = 0; d < DIM; d++)
        if ((x[d]-raster[d][idx[d]])/(raster[d][idx[d]+1]-raster[d][idx[d]]) > 0.5)
            idx[d]++;

    // Convert the cell index to a flat index.
    int n = getFlatIndex(idx);

    // Append to the data bin.
    if (maxBinSize > 0)
        while (bin[n].size() > maxBinSize-1)
            bin[n].pop_front();
    bin[n].push_back(NVec<4>(x.x, x.y, x.z, y));

    // Calculate the new centroid.
    // I should replace this by a running average, or running median.
    centroid[n] = 0;
    for (int l = 0; l < bin[n].size(); l++)
        centroid[n] += bin[n][l].last();
    centroid[n] /= (double)bin[n].size();

    // Activate the neighbourhood of the cell that has been touched.
//    QVector<int> nn = enumerateNeighborHood(idx, activationRadius);
//    foreach (int n, nn)
//        active[n] = true;
}

// Returns true if the given point is within the (transformed) boundaries of the grid.
bool GPRGRID3D::containsPoint(NVec<3> x)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    x = NVec<3>(trx.x, trx.y, trx.z);

    for (int d = 0; d < DIM; d++)
        if (x[d] > max[d] || x[d] < min[d])
            return false;

    return true;
}

// Erases all data from the grid cell identified by the DIM index idx.
void GPRGRID3D::eraseData(const int* idx)
{
    int n = getFlatIndex(idx);
    bin[n].clear();
    centroid[n] = 0;
}

// Erases all data from the grid cell identified by the flat index n.
void GPRGRID3D::eraseData(int n)
{
    bin[n].clear();
    centroid[n] = 0;
}

// Returns the data points in a neighbourhood of radius r
// around the grid node identified by the DIM index idx.
QList< NVec<4> > GPRGRID3D::getData(const int *idx, int r)
{
    QList< NVec<4> > data;
    QVector<int> nn = enumerateNeighborHood(idx, r);
    for (int i=0; i < nn.size(); i++)
        data << bin[nn[i]];
    return data;
}

// Returns the data points in a neighbourhood of radius r
// around the grid node identified by the flat index n.
QList< NVec<4> > GPRGRID3D::getData(int n, int r)
{
    QList< NVec<4> > data;
    QVector<int> nn = enumerateNeighborHood(n, r);
    for (int i=0; i < nn.size(); i++)
        data << bin[nn[i]];
    return data;
}

// Returns the data centroids in a neighbourhood of radius r
// around the grid node identified by the DIM index idx.
// Only centroids are returned of bins that have at least
// on data point in them.
QList<double> GPRGRID3D::getCentroids(const int *idx, int r)
{
    QList<double> data;
    QVector<int> nn = enumerateNeighborHood(idx, r);
    for (int i=0; i < nn.size(); i++)
        if (!bin[nn[i]].isEmpty())
            data << centroid[nn[i]];
    return data;
}

// Returns the data centroids in a neighbourhood of radius r
// around the grid node identified by the flat index n.
// Only centroids are returned of bins that have at least
// on data point in them.
QList<double> GPRGRID3D::getCentroids(int n, int r)
{
    QList<double> data;
    QVector<int> nn = enumerateNeighborHood(n, r);
    for (int i=0; i < nn.size(); i++)
        if (!bin[nn[i]].isEmpty())
            data << centroid[nn[i]];
    return data;
}

// Activates the grid node identified by the flat index n.
void GPRGRID3D::activate(int n)
{
    active[n] = true;
}

// Set the grid node outputs to the centroids of the data bins whereever applicable.
void GPRGRID3D::setNodeOutputsToDataCentroids()
{
    // This algorithm could be much more efficient if the data bins were a hash
    // so that I could loop over the hash keys of non empty bins, rather than
    // looping over the entire grid and checking if the bins have data in it.
    for (int n = 0; n < nodeCount; n++)
    {
        if (bin[n].size() > 0)
        {
            Y[n] = centroid[n];
            C[n] = 1;
        }
    }
}


// Performs a local neighborhood based training step.
void GPRGRID3D::train()
{
    GPR<3> gpr;
    gpr.setMinMax(min, max);
    gpr.p1 = config.gprP1;
    gpr.p2 = config.gprP2; // kernel width

    for (int n = 0; n < nodeCount; n++)
    {
        QVector<int> nn = enumerateNeighborHood(n, activationRadius);

        QList< NVec<4> > data;
        for (int i=0; i < nn.size(); i++)
            data << bin[nn[i]];

        QList<NVec<3> > X;
        QList<double> y;
        for (int i = 0; i < data.size(); i++)
        {
            X << NVec<3>(data[i].x, data[i].y, data[i].z);
            y << data[i].w;
        }
        gpr.clear();
        gpr.addData(X, y);

        NVec<DIM> x = GRID<3>::getNodeCoordinates(n);
        double mean = 0;
        double conf = 0;
        gpr.evaluateAt(x, mean, conf);
        setNodeOutput(n, mean, 1.1-conf);

        qDebug() << n << "of" << nodeCount << "data points" << data.size() << "radius" << activationRadius << "nn size" << nn.size() << "mean:" << mean << "conf:" << conf;
    }
}


// Computes the GPR using all data in the grid.
void GPRGRID3D::trainAllData()
{
    GPR<3> gpr;
    gpr.setMinMax(min, max);
    gpr.p1 = config.gprP1;
    gpr.p2 = config.gprP2; // kernel width

    QList< NVec<4> > data;
    for (int n = 0; n < nodeCount; n++)
        data << bin[n];
    qDebug() << data.size() << "data points identified for training.";

    QList<NVec<3> > X;
    QList<double> y;
    for (int i = 0; i < data.size(); i++)
    {
        X << NVec<3>(data[i].x, data[i].y, data[i].z);
        y << data[i].w;
    }
    gpr.addData(X, y);

    for (int n = 0; n < nodeCount; n++)
    {
        NVec<DIM> x = GRID<3>::getNodeCoordinates(n);
        double mean = 0;
        double conf = 0;
        gpr.evaluateAt(x, mean, conf);
        setNodeOutput(n, mean, 1.1-conf);

        if (n % 100 == 0)
            qDebug() << n << "of" << nodeCount << "mean:" << mean << "conf:" << conf;

    }
}

// OpenGL drawing code that draws a 2D slice of the grid.
// No interpolation is used, just the output values at the grid coordinates.
// The input parameter should be in [0,1]. It will be mapped to the "vertical"
// index of the grid slice. The drawing does not apply the transform.
void GPRGRID3D::drawGridSlice(double z, int sampleFactor, double zScale, double colorContrast, double transparency, bool showData)
{
    GRID3D::drawGridSlice(z, sampleFactor, zScale, colorContrast, transparency, showData);

    if (showData)
    {
        // Compute "vertical" slice index.
        int k = qBound(0, int(z*N[2]-1), N[2]-1);
        sampleFactor = qMax(sampleFactor, 1);

        glPushMatrix();
        int idx[3] = {0,0,k};
        glTranslated(0, 0, GRID<3>::getNodeCoordinates(idx).z);
        glScaled(1.0, 1.0, zScale);

        // Draw the data points.
        glPointSize(4);
        glColor3f(0,0,0);
        glBegin(GL_POINTS);
        for (int j = 0; j < N[1]; j++)
        {
            for (int i = 0; i < N[0]; i++)
            {
                int idx[3] = {i,j,k};
                int n = getFlatIndex(idx);
                for (int l = 0; l < bin[n].size(); l++)
                    glVertex3f(bin[n][l].x, bin[n][l].y, bin[n][l].w);
            }
        }
        glEnd();

        // Mark the active neurons with white.
        glBegin(GL_POINTS);
        glColor3f(1,1,1);
        QList<int> keys = active.keys();
        for (int i = 0; i < keys.size(); i++)
        {
            QVector<int> idx = getDimIndex(keys[i]);
            if (idx[2] == k)
                glVertex3f(raster[0][idx[0]], raster[1][idx[1]], Y[keys[i]]);
        }
        glEnd();

        glPopMatrix();

    }
}

// OpenGL drawing code that draws the location of the data points that are
// stored in the grid.
void GPRGRID3D::drawData()
{
    GLlib::drawWireBox(max.x, max.y, max.z);

    QColor color;
    glPointSize(2);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    for (int i = 0; i < bin.size(); i++)
    {
        for (int j = 0; j <  bin[i].size(); j++)
        {
            color = colorUtil.mapColor(bin[i][j].w, 0, config.colorContrast);
            glColor3f(color.redF(), color.greenF(), color.blueF());
            glVertex3f(bin[i][j].x, bin[i][j].y, bin[i][j].z);
        }
    }
    glEnd();
}
