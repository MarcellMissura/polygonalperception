#include "Som3D.h"
#include "globals.h"
#include "GPR.h"
#include "GL/gl.h"
#include <util/ColorUtil.h>
#include "util/GLlib.h"
#include "util/FileLoader.h"
#include "framework/Config.h"
#include <QFile>
#include <QString>
#include <QStringList>
#include <vector>
#include <QGLViewer/vec.h>
#include <QGLViewer/quaternion.h>
using namespace qglviewer;


// The SOM neural network is a simple learner for 3D input dimensions and 1 output dimension.
// The network is a lattice evenly spread out in the data range. The nodes (neurons) of the lattice
// are pulled towards the centroid of the data in the vicinity, but also towards the centroid of
// their neighborhood neurons.

Som3D::Som3D()
{
	dataGain = 1.0; // How strongly the som neurons are pulled towards the data centroids.
    neighborhoodGain = 1.0; // How strongly the som neurons are pulled towards their neighborhood centroids.
    timestep = 0.5; // Simulation time step. The larger the faster, but the more likely to explode.
	maxBinSize = 20;  // The maximum number of points allowed to be in a bin. Older points are discarded. 0 means no limit.
    blurKernelWidth = 1; // The width of the smoothing kernel.
    activationRadius = 1;
}

// Sets the data gain that determines how strongly the neurons are pulled into the data centroids.
// Can be changed any time, even during training.
void Som3D::setDataGain(double dataGain)
{
	this->dataGain = dataGain;
}

// Sets the neighborhood gain that determines how strongly the neurons are pulled towards the centroid of their neighbors.
// Can be changed any time, even during training.
void Som3D::setNeighborhoodGain(double neighborhoodGain)
{
	this->neighborhoodGain = neighborhoodGain;
}

// Sets the time step parameter. The timestep parameter determines how far the neurons are moved per training iteration.
// The higher the time step, the faster the training converges, but if the number is too high, the network will explode.
// Can be changed any time, even during training.
void Som3D::setTimestep(double timestep)
{
	this->timestep = timestep;
}

// Sets the maximum number of allowed data points in a bin.
// Can be changed any time, even during training.
void Som3D::setMaxBinSize(int maxBinSize)
{
	this->maxBinSize = maxBinSize;
}

// Sets the width of the smoothing kernel.
void Som3D::setBlurKernelWidth(int kw)
{
    blurKernelWidth = kw;
}

// Sets the radius of the neighbourhood that gets activated around each
// touched grid node when new data arrive.
void Som3D::setActivationRadius(int ar)
{
    activationRadius = ar;
}


// Sets N, the number of nodes per dimension. This method sets N to be the same
// for every dimension and thus creates a uniform grid.
void Som3D::setN(int N_)
{
    GRID3D::setN(N_);

    int nodeCount = getNodeCount();
    bin.resize(nodeCount);
    centroid.resize(nodeCount);
    centroidForce.resize(nodeCount);
    firstOrderForce.resize(nodeCount);
    filterBuffer.resize(nodeCount);
}

// Sets N, the number of nodes per dimension. This method sets N[d] for each dimension individually.
// The size of the array is DIM, of course.
void Som3D::setN(const int* N_)
{
    GRID3D::setN(N_);

    int nodeCount = getNodeCount();
    bin.resize(nodeCount);
    centroid.resize(nodeCount);
    centroidForce.resize(nodeCount);
    firstOrderForce.resize(nodeCount);
    filterBuffer.resize(nodeCount);
}

// Prints a list of the data stored in the som.
void Som3D::printData()
{
    for (int i = 0; i < bin.size(); i++)
        for (int j = 0; j < bin[i].size(); j++)
            qDebug() << bin[i][j];
}

// Clears the data bins, but does not change the output values.
void Som3D::clearData()
{
    for (int i = 0; i < bin.size(); i++)
        bin[i].clear();
}

// Batch adds a bunch of points.
void Som3D::addData(QList< NVec<3> > X, QList<double> y)
{
    for (int i = 0; i < X.size(); i++)
        addDataPoint(X[i], y[i]);
}

// Adds a single data point with input values x and output value y to the data set.
void Som3D::addDataPoint(NVec<3> x, double y)
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
    QVector<int> nn = enumerateNeighborHood(idx, activationRadius);
    foreach (int n, nn)
        active[n] = true;
}

// Returns true if the given point is within the (transformed) boundaries of the grid.
bool Som3D::containsPoint(NVec<3> x)
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
void Som3D::eraseData(const int* idx)
{
    int n = getFlatIndex(idx);
    bin[n].clear();
    centroid[n] = 0;
}

// Erases all data from the grid cell identified by the flat index n.
void Som3D::eraseData(int n)
{
    bin[n].clear();
    centroid[n] = 0;
}

// Returns the data points in a neighbourhood of radius r
// around the grid node identified by the DIM index idx.
QList< NVec<4> > Som3D::getData(const int *idx, int r)
{
    QList< NVec<4> > data;
    QVector<int> nn = enumerateNeighborHood(idx, r);
    for (int i=0; i < nn.size(); i++)
        data << bin[nn[i]];
    return data;
}

// Returns the data points in a neighbourhood of radius r
// around the grid node identified by the flat index n.
QList< NVec<4> > Som3D::getData(int n, int r)
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
QList<double> Som3D::getCentroids(const int *idx, int r)
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
QList<double> Som3D::getCentroids(int n, int r)
{
    QList<double> data;
    QVector<int> nn = enumerateNeighborHood(n, r);
    for (int i=0; i < nn.size(); i++)
        if (!bin[nn[i]].isEmpty())
            data << centroid[nn[i]];
    return data;
}

// Performs a bunch of training steps.
void Som3D::train(int howManySteps)
{
	for (int i = 0; i < howManySteps; i++)
    {
        blur();
        setNodeOutputsToDataCentroids();
    }
}

// Activates the grid node identified by the flat index n.
void Som3D::activate(int n)
{
    active[n] = true;
}

// Set the grid node outputs to the centroids of the data bins whereever applicable.
void Som3D::setNodeOutputsToDataCentroids()
{
    // This algorithm could be much more efficient if the data bins were a hash
    // so that I could loop over the hash keys of non empty bins, rather than
    // looping over the entire grid and checking if the bins have data in it.
    for (int n = 0; n < nodeCount; n++)
        if (bin[n].size() > 0)
            Y[n] = centroid[n];
}

// Performs a GPR based training step.
void Som3D::gprTraining()
{
    GPR<3> gpr;
    gpr.setMinMax(min, max);
    gpr.p1 = config.gprP1;
    gpr.p2 = config.gprP2;

    QList<int> keys = active.keys();
    for (int i = 0; i < keys.size(); i++)
    {
        int n = keys[i];
        QVector<int> nn = enumerateNeighborHood(n, activationRadius);

        QList< NVec<4> > data;
        for (int i=0; i < nn.size(); i++)
            data << bin[nn[i]];
        qDebug() << i << "of" << keys.size() << "data" << data.size() << "radius" << activationRadius << "nn size" << nn.size();

        QList<NVec<3> > X;
        QList<double> y;
        for (int i = 0; i < data.size(); i++)
        {
            X << NVec<3>(data[i].x, data[i].y, data[i].z);
            y << data[i].w;
        }
        gpr.clear();
        gpr.addData(X, y);

        NVec<DIM> x = getNodeCoordinates(n);
        double mean = 0;
        gpr.evaluateAt(x, mean);
        setNodeOutput(n, mean);
    }

    active.clear();
}

// Applies an efficient blur filter to smooth the output values of the grid nodes.
// The blurKernelWidth parameter is used as the kernel radius.
// The blur filter is implemented as a sequence of one dimensional box filters,
// each performed on the result of the previus runs.
// See https://en.wikipedia.org/wiki/Gaussian_blur
// and http://blog.ivank.net/fastest-gaussian-blur.html
void Som3D::blur()
{
    // The kernel radius must be smaller than the min N.
    int minN = N[0];
    for (int d=1; d<DIM; d++)
        minN = qMin(minN, N[d]);
    int r = qBound(1, blurKernelWidth, int(minN/2)-1);

    QVector<double> *src;
    QVector<double> *trgt;
    QVector<double> *tmp;

    src = &Y;
    trgt = &filterBuffer;

    // For every dimension...
    for (int d = 0; d < DIM; d++)
    {
        // Number of starting points to run a 1D motion blur from.
        int startNodeCount = 1;
        for (int dd = 0; dd < DIM; dd++)
            if (dd != d)
                startNodeCount *= N[dd];

        // Now loop over all starting points and perform the motion blur along the relevant dimension.
        for (int snc = 0; snc < startNodeCount; snc++)
        {
            // Convert snc to a DIM dimensional index with the relevant dimension set to 0 .
            int dimidx[DIM];
            int v = snc;
            for (int dd = 0; dd < DIM; dd++)
            {
                if (dd == d)
                {
                    dimidx[d] = 0; // Inject 0 at the relevant dimension.
                }
                else
                {
                    dimidx[dd] = v % N[dd];
                    v = int(v/N[dd]);
                }
            }

            // Compute the flat index n of the current start point.
            int n = getFlatIndex(dimidx);

            // And now perform the 1D motion blur.
            // I handle the boundary so that I start with a kernel of half size,
            // I grow it while I move away from the border, run it at full size
            // until it touches the end border, and shrink it down again.

            // init kernel
            int c = r; // c is the number of cells in the kernel
            double runSum = 0; // runSum is the sum of all cells in the kernel
            for (int ii = 0; ii < r; ii++)
                runSum += (*src)[n+ii*cumN[d]];

            // grow kernel
            for (int i = 0; i <= r; i++)
            {
                runSum += (*src)[n+r*cumN[d]];
                c++;
                (*trgt)[n] = runSum/c;
                n += cumN[d];
            }

            // run kernel
            for (int i = r+1; i < N[d]-r; i++)
            {
                runSum += (*src)[n+r*cumN[d]]-(*src)[n-(r+1)*cumN[d]];
                (*trgt)[n] = runSum/c;
                n += cumN[d];
            }

            // shrink kernel
            for (int i = N[d]-r; i < N[d]; i++)
            {
                runSum -= (*src)[n-(r+1)*cumN[d]];
                c--;
                (*trgt)[n] = runSum/c;
                n += cumN[d];
            }
        }

        // Flip buffer.
        tmp = src;
        src = trgt;
        trgt = src;
    }

    // If the number of dimensions is odd, the buffer needs to be copied into Y.
    if (DIM % 2 == 1)
        memcpy(Y.data(), filterBuffer.data(), nodeCount*sizeof(double));
}

// Calculates the forces acting on the neurons and moves the neurons by a bit.
void Som3D::oldTrainingStep()
{
/*
	// Calculate centroid forces.
    for (int n = 0; n < nodeCount; n++)
        centroidForce[n] = bin[n].size() > 0 ? centroid[n] - Y[n] : 0;

	// Calculate first order neighborhood forces with explicit border case handling.
	// Core.
    for (int i = 1; i < N[0]-1; i++)
    {
        for (int j = 1; j < N[1]-1; j++)
        {
            for (int k = 1; k < N[2]-1; k++)
            {
                int n = i*cumN[0] + j*cumN[1] + k*cumN[2];
                firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/6.0 - Y[n];
            }
        }
    }

	// Faces.
    for (int i = 1; i < N[0]-1; i++)
	{
        for (int j = 1; j < N[1]-1; j++)
		{
            int n = i*cumN[0] + j*cumN[1] + 0*cumN[2];
            firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n+cumN[2]])/5.0 - Y[n];

            n = i*cumN[0] + j*cumN[1] + (N[2]-1)*cumN[2];
            firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n-cumN[2]])/5.0 - Y[n];
		}
	}

    for (int i = 1; i < N[0]-1; i++)
    {
        for (int k = 1; k < N[2]-1; k++)
        {
            int n = i*cumN[0] + 0*cumN[1] + k*cumN[2];
            firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/5.0 - Y[n];

            n = i*cumN[0] + (N[1]-1)*cumN[1] + k*cumN[2];
            firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/5.0 - Y[n];
        }
    }

    for (int j = 1; j < N[1]-1; j++)
    {
        for (int k = 1; k < N[2]-1; k++)
        {
            int n = 0*cumN[0] + j*cumN[1] + k*cumN[2];
            firstOrderForce[n] = (Y[n+cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/5.0 - Y[n];

            n = (N[0]-1)*cumN[0] + j*cumN[1] + k*cumN[2];
            firstOrderForce[n] = (Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/5.0 - Y[n];
        }
    }

	// Edges.
    for (int i = 1; i < N[0]-1; i++)
	{
        int n = i*cumN[0] + 0*cumN[1] + 0*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n+cumN[2]])/4.0 - Y[n];

        n = i*cumN[0] + (N[1]-1)*cumN[1] + 0*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[2]])/4.0 - Y[n];

        n = i*cumN[0] + (N[1]-1)*cumN[1] + (N[2]-1)*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n-cumN[1]] + Y[n-cumN[2]])/4.0 - Y[n];

        n = i*cumN[0] + 0*cumN[1] + (N[2]-1)*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[2]])/4.0 - Y[n];
	}

    for (int j = 1; j < N[1]-1; j++)
    {
        int n = 0*cumN[0] + j*cumN[1] + 0*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n+cumN[2]])/4.0 - Y[n];

        n = (N[0]-1)*cumN[0] + j*cumN[1] + 0*cumN[2];
        firstOrderForce[n] = (Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n+cumN[2]])/4.0 - Y[n];

        n = (N[0]-1)*cumN[0] + j*cumN[1] + (N[2]-1)*cumN[2];
        firstOrderForce[n] = (Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[1]] + Y[n-cumN[2]])/4.0 - Y[n];

        n = 0*cumN[0] + j*cumN[1] + (N[2]-1)*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[1]] + Y[n-cumN[2]])/4.0 - Y[n];
    }

    for (int k = 1; k < N[2]-1; k++)
    {
        int n = 0*cumN[0] + 0*cumN[1] + k*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n+cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/4.0 - Y[n];

        n = (N[0]-1)*cumN[0] + 0*cumN[1] + k*cumN[2];
        firstOrderForce[n] = (Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/4.0 - Y[n];

        n = (N[0]-1)*cumN[0] + (N[1]-1)*cumN[1] + k*cumN[2];
        firstOrderForce[n] = (Y[n-cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[2]] + Y[n-cumN[2]])/4.0 - Y[n];

        n = 0*cumN[0] + (N[1]-1)*cumN[1] + k*cumN[2];
        firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[1]] + Y[n-cumN[2]])/4.0 - Y[n];
    }

	// Corners.
    int n = 0*cumN[0] + 0*cumN[1] + 0*cumN[2];
    firstOrderForce[n] = (Y[n+cumN[0]] + Y[n+cumN[1]] + Y[n+cumN[2]])/3.0 - Y[n];

    n = (N[0]-1)*cumN[0] + 0*cumN[1] + 0*cumN[2];
    firstOrderForce[n] = (Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n+cumN[2]])/3.0 - Y[n];

    n = 0*cumN[0] + (N[1]-1)*cumN[1] + 0*cumN[2];
    firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[2]])/3.0 - Y[n];

    n = 0*cumN[0] + 0*cumN[1] + (N[2]-1)*cumN[2];
    firstOrderForce[n] = (Y[n+cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[2]])/3.0 - Y[n];

    n = (N[0]-1)*cumN[0] + (N[1]-1)*cumN[1] + 0*cumN[2];
    firstOrderForce[n] = (Y[n-cumN[0]] + Y[n-cumN[1]] + Y[n+cumN[2]])/3.0 - Y[n];

    n = (N[0]-1)*cumN[0] + 0*cumN[1] + (N[2]-1)*cumN[2];
    firstOrderForce[n] = (Y[n-cumN[0]] + Y[n+cumN[1]] + Y[n-cumN[2]])/3.0 - Y[n];

    n = 0*cumN[0] + (N[1]-1)*cumN[1] + (N[2]-1)*cumN[2];
    firstOrderForce[n] = (Y[n+cumN[0]] + Y[n-cumN[1]] + Y[n-cumN[2]])/3.0 - Y[n];

    n = (N[0]-1)*cumN[0] + (N[1]-1)*cumN[1] + (N[2]-1)*cumN[2];
    firstOrderForce[n] = (Y[n-cumN[0]] + Y[n-cumN[1]] + Y[n-cumN[2]])/3.0 - Y[n];

	// Update neuron states with a simple epsilon * target algorithm.
    for (int n = 0; n < nodeCount; n++)
    {
        double update = timestep * (dataGain*centroidForce[n] + neighborhoodGain*firstOrderForce[n]);

//        if (centroidForce[n] > 0.000001)
//            qDebug() << "moving" << n << "update:" << update << "cf:" << centroidForce[n] << "nf:" << firstOrderForce[n];
        Y[n] += update;
    }
*/
}


// OpenGL drawing code that draws a 2D slice of the grid.
// No interpolation is used, just the output values at the grid coordinates.
// The input parameter should be in [0,1]. It will be mapped to the "vertical"
// index of the grid slice. The drawing does not apply the transform.
void Som3D::drawGridSlice(double z, int sampleFactor, double zScale, double colorContrast, bool showData)
{
    GRID3D::drawGridSlice(z, sampleFactor, zScale, colorContrast, showData);


    if (showData)
    {
        // Compute "vertical" slice index.
        int k = qBound(0, int(z*N[2]-1), N[2]-1);
        sampleFactor = qMax(sampleFactor, 1);

        glPushMatrix();
        int idx[3] = {0,0,k};
        glTranslated(0, 0, getNodeCoordinates(idx).z);
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
void Som3D::drawData()
{
    GLlib::drawWireBox(max.x, max.y, max.z);

    glPointSize(4);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    for (int i = 0; i < bin.size(); i++)
        for (int j = 0; j <  bin[i].size(); j++)
            glVertex3f(bin[i][j].x, bin[i][j].y, bin[i][j].z);
    glEnd();
}
