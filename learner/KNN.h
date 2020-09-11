#ifndef KNN_H_
#define KNN_H_
#include <QFile>
#include "util/VecN.h"
#include "util/ColorUtil.h"
#include "framework/Config.h"
#include <ANN/ANN.h>
#include "armadillo"
#include <QGLViewer/vec.h>
#include <QGLViewer/quaternion.h>
#include <QGLViewer/frame.h>

// This is a simple KNN retriever class.
// The input and output dimensions are specified as template parameters.
// The input dimensions (DIMIN) span the data space where the actual KNN search is performed.
// The output dimensions(DIMOUT) are just additional values that are attached to each data point.
// Use addDataPoint() to feed the KNN with data.
// Then, use init() to initialze a search structure (kd-tree) suitable for Euklidean search.
// After initialization, you can use retrieve() to query for the k nearest neighbours.

template <int DIMIN=3, int DIMOUT=1>
class KNN
{
    int loadedPoints; // number of actually loaded points
    bool initialized; // status check flag

    VecN<DIMIN> mean;
    VecN<DIMIN> max;
    VecN<DIMIN> min;
    QVector<double> data;
    ANNpointArray dataPoints; // data points
    ANNkd_tree* kdTree; // search structure

    double scale;

public:

    KNN();
    ~KNN();

    void init();
    void reset();

    int getLoadedPointCount();
    void setMean(VecN<DIMIN> m);

    VecN<DIMIN> getMean();
    VecN<DIMIN> getMax();
    VecN<DIMIN> getMin();

    void load(QString name = "data/KNN.knn");
    void save(QString name = "data/KNN.knn");

    void addDataPoint(const VecN<DIMIN+DIMOUT> x);

    const QList<VecN<DIMIN+DIMOUT+1> > knnSearch(VecN<DIMIN> x, int k=1, double epsilon=0);
    const QList<VecN<DIMIN+DIMOUT+1> > radiusSearch(VecN<DIMIN> x, double r, int k=0, double epsilon=0);

    VecN<DIMIN+DIMOUT> operator[](int i);

    // OpenGL drawing code.
    void setScale(double s);
    void draw(uint sampleFactor=1);
};

// Boring constructor.
template <int DIMIN, int DIMOUT>
KNN<DIMIN, DIMOUT>::KNN()
{
    loadedPoints = 0;
    kdTree = 0;
    scale = 1.0;
}

// Destructor.
template <int DIMIN, int DIMOUT>
KNN<DIMIN, DIMOUT>::~KNN()
{
    if (kdTree != 0)
        delete kdTree;
    annClose();
}

// Clears all data points and resets the knn search to a blank state.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::reset()
{
    loadedPoints = 0;
    data.clear();
}

// Sets the mean. It only has an influence on the visualization.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::setMean(VecN<DIMIN> m)
{
    mean = m;
}

// Returns the mean.
template <int DIMIN, int DIMOUT>
VecN<DIMIN> KNN<DIMIN, DIMOUT>::getMean()
{
    return mean;
}

// Returns the max.
template <int DIMIN, int DIMOUT>
VecN<DIMIN> KNN<DIMIN, DIMOUT>::getMax()
{
    return max;
}

// Returns the min.
template <int DIMIN, int DIMOUT>
VecN<DIMIN> KNN<DIMIN, DIMOUT>::getMin()
{
    return min;
}

// Returns the number of points in the data set.
template <int DIMIN, int DIMOUT>
int KNN<DIMIN, DIMOUT>::getLoadedPointCount()
{
    return loadedPoints;
}

// Initializes the knn search for use with the Euklidean metric.
// This method should be called after all data points have been added.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::init()
{
    if (loadedPoints == 0)
        return;

    // Set up the data pointers for easier handling.
    // And also ANN wants to see the data this way.
    dataPoints = new ANNpoint[loadedPoints];
    double* dataPtr = data.data();
    for (int i = 0; i < loadedPoints; i++)
        dataPoints[i] = &(dataPtr[i*(DIMIN+DIMOUT)]);

    // Compute min, max, mean.
    mean = 0;
    for (int d=0; d<DIMIN; d++)
    {
        max[d] = dataPoints[0][d];
        min[d] = dataPoints[0][d];
    }
    for (int k = 0; k < loadedPoints; k++)
    {
        for (int d=0; d<DIMIN; d++)
        {
            max[d] = qMax(max[d], dataPoints[k][d]);
            min[d] = qMin(min[d], dataPoints[k][d]);
            mean[d] += dataPoints[k][d];
        }
    }
    mean /= loadedPoints;

    // Build the kd-tree.
    if (kdTree != 0)
        delete kdTree;
    kdTree = new ANNkd_tree(dataPoints, loadedPoints, DIMIN);
    initialized = true;
}

// Retrieves the k nearest neighbours of x from the data set.
// Nearest neighbours are searched in the DIMIN dimensional input space.
// The Euklidean norm is used as metric used to compute distances.
// The returned data points are DIMIN+DIMOUT long and contain the input coordinates as well as the output.
// Set k to indicate how many neighbours you want.
// Set epsilon > 0 to allow an error margin that speeds up the search.
// You must add data points and call init() before calling this method.
template <int DIMIN, int DIMOUT>
const QList<VecN<DIMIN+DIMOUT+1> > KNN<DIMIN, DIMOUT>::knnSearch(VecN<DIMIN> x, int k, double epsilon)
{
    QList<VecN<DIMIN+DIMOUT+1> > knn;

    if (!initialized)
        return knn;

    // Prepare the query point.
    ANNpoint queryPt = annAllocPt(DIMIN); // allocate query point
    for (int d = 0; d < DIMIN; d++)
        queryPt[d] = x[d];

    // Allocate the return structures.
    ANNidxArray nnIdx = new ANNidx[k]; // near neigh indices
    ANNdistArray dists = new ANNdist[k]; // near neighbor dists

    kdTree->annkSearch(
                    queryPt,	// query point
                    k,          // number of near neighbors
                    nnIdx,      // nearest neighbors indices (returned)
                    dists,      // squared distances (returned)
                    epsilon);   // error bound

    // Attach the distance to the data points and collect them in a list.
    for (int i=0; i<k; i++)
    {
        VecN<DIMIN+DIMOUT+1> v(dataPoints[nnIdx[i]], sqrt(dists[i]));
        knn << v;
    }

    delete nnIdx;
    delete dists;

    return knn;
}

// Retrieves the k nearest neighbours of x that are within a radius r.
// Set k=0 to retrieve ALL neighbours within the radius r.
// Nearest neighbours are searched in the DIMIN dimensional input space.
// The Euklidean norm is used as metric to compute distances.
// The returned data points are DIMIN+DIMOUT long and contain the input coordinates as well as the output.
// Set k to indicate how many neighbours you want.
// Set epsilon > 0 to allow an error margin that speeds up the search.
// You must add data points and call init() before calling this method.
template <int DIMIN, int DIMOUT>
const QList<VecN<DIMIN+DIMOUT+1> > KNN<DIMIN, DIMOUT>::radiusSearch(VecN<DIMIN> x, double r, int k, double epsilon)
{
    QList<VecN<DIMIN+DIMOUT+1> > knn;

    if (!initialized)
        return knn;

    // Prepare the query point.
    ANNpoint queryPt = annAllocPt(DIMIN); // allocate query point
    for (int d = 0; d < DIMIN; d++)
        queryPt[d] = x[d];

    // If k = 0, all points in the radius are retrieved.
    if (k==0)
        k = kdTree->annkFRSearch(queryPt, r*r, 0);

    // Allocate the return structures.
    ANNidxArray nnIdx = new ANNidx[k]; // near neigh indices
    ANNdistArray dists = new ANNdist[k]; // near neighbor dists

    kdTree->annkFRSearch(queryPt, // query point ANNdist
                         r*r, // squared radius
                         k, // number of near neighbors to return
                         nnIdx, // nearest neighbor array (modified)
                         dists, // dist to near neighbors (modified)
                         epsilon); // error bound

    // Attach distances to the retrieved points.
    for (int i=0; i<k; i++)
    {
        if (nnIdx[i] == ANN_NULL_IDX)
            break;

        VecN<DIMIN+DIMOUT+1> v(dataPoints[nnIdx[i]], sqrt(dists[i]));
        knn << v;
    }

    delete nnIdx;
    delete dists;

    return knn;
}

// Returns the loaded point at index i.
template <int DIMIN, int DIMOUT>
VecN<DIMIN+DIMOUT> KNN<DIMIN, DIMOUT>::operator[](int i)
{
    return VecN<DIMIN+DIMOUT>(dataPoints[i]);
}


// Adds a single point to the data point set.
// Do this first and then call init(). After calling init(), do not add
// any more points. If you have to, call clear() and readd all points.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::addDataPoint(const VecN<DIMIN+DIMOUT> x)
{
    initialized = false;
    for (int d = 0; d < DIMIN+DIMOUT; d++)
        data << x[d];
    loadedPoints++;
}

// Saves the KNN in a binary file.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::save(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "KNN::save(): invalid file name" << name;
        return;
    }
    fileName += ".knn";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "KNN::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << loadedPoints;
    out << data;
    file.close();
}

// Loads a binary saved KNN.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::load(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "KNN::load(): invalid file name" << name;
        return;
    }
    fileName += ".knn";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "KNN::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> loadedPoints;
    in >> data;
    file.close();
    init();
}

// Sets the scaling factor.
// The scaling factor is used only for visualization!
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::setScale(double s)
{
    scale = s;
}

// OpenGL drawing code that draws the location of the stored data points.
template <int DIMIN, int DIMOUT>
void KNN<DIMIN, DIMOUT>::draw(uint sampleFactor)
{
    if (!initialized)
        return;

    QColor color;
    double colorMin = 0;
    double colorMax = config.colorContrast;

    glPushMatrix();
    glPointSize(4);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    sampleFactor = qMax((uint)1, sampleFactor);
    for (int k=0; k < loadedPoints; k=k+sampleFactor)
    {
        double t = fabs(dataPoints[k][3])+fabs(dataPoints[k][4])+fabs(dataPoints[k][5]);
        color = colorUtil.getHeightMapColor(t, colorMin, colorMax);
        glColor3f(color.redF(), color.greenF(), color.blueF());
        glVertex3d(dataPoints[k][0], dataPoints[k][1], dataPoints[k][2]);
    }
    glEnd();
    glPopMatrix();
}

#endif /* KNN_H_ */
