#include "LLKR.h"
#include "globals.h"
#include "util/GLlib.h"
#include "util/ColorUtil.h"
#include "util/Statistics.h"
#include "learner/OLS.h"
#include "framework/Config.h"
#include "framework/State.h"
#include "pml/poc.h"
#include "QGLViewer/vec.h"
#include "QGLViewer/quaternion.h"
#include <QFile>

// Locally Linear Kernel Regression
// This class implements a locally linear kernel regressor.
// The regressor is specific to the pole cart model for now.
// When you add data points using addPoint(x,y), you provide the mapping y
// that result in state x. When adding a point, the routine computes a linear
// transformation from a local basis in Y (control space) to a local basis
// in X (state space). When querying the nearest neighbours for a query point x,
// the query point is transformed into the local frame of every saved kernel (point)
// before its distance to the origin of the kernel is computed. Nearest neighbours
// are returned by that distance. Additionally, each kernel resports an estimate
// output value for the query point x, which is computed from the linear expansion
// of the kernel to the query point.

// Boring constructor.
LLKR::LLKR()
{
    initialized = false;
}

// Destructor.
LLKR::~LLKR()
{

}

// Clears all data points and resets the knn search to a blank state.
void LLKR::reset()
{
    QMutexLocker locker(&mutex);
    data.clear();
    initialized = false;
}

// Sets the global transform T. All data points x are transformed with
// the inverse of the global transform. A good choice for this transform
// is the eigenvalue decomposition of the covariance matrix given to you
// by the computePrincipalTransform() function. Make sure to set the
// tranform before calling init()!
void LLKR::setTransform(NMat<3, 3> T)
{
    this->T = T;
    this->Tinv = T.inverse();
}

// Initializes a kd-tree for knn search with the Euklidean metric.
// This method should be called after all data points have been added.
void LLKR::init()
{
    if (data.size() == 0)
        return;

    // Transform all points.
    transformedX.clear();
    for (int i = 0; i < data.size(); i++)
        transformedX << Tinv*data[i].x;

    // Set up the data pointers for easier handling.
    // And also ANN wants to see the data this way.
    ANNpointArray dataPointsX = new ANNpoint[transformedX.size()];
    for (int i = 0; i < transformedX.size(); i++)
        dataPointsX[i] = transformedX[i].data();

    ANNpointArray dataPointsZ = new ANNpoint[data.size()];
    for (int i = 0; i < data.size(); i++)
        dataPointsZ[i] = data[i].z.data();

    // Build the kd-trees.
//    if (kdTreeX != 0)
//        delete kdTreeX; // Why this no work??
    kdTreeX = new ANNkd_tree(dataPointsX, data.size(), 3);

//    if (kdTreeY != 0)
//        delete kdTreeY;
    kdTreeZ = new ANNkd_tree(dataPointsZ, data.size(), 3);

    initialized = true;
}

// Sorts the dataset into ascending time order.
// Attention, it rewrites the ids!
void LLKR::sort()
{
    std::sort(data.begin(), data.end());
    for (int i = 0; i < data.size(); i++)
        data[i].id = i;
}

// Computes all kernel data.
void LLKR::computeAllKernels()
{
    for (int i = 0; i < data.size(); i=i+2)
    {
        if (!data[i].isKnown())
        {
            data[i].computeKernelData();
            setKernelData(i+1, -data[i].Jinv, data[i].errorRate);
        }

        if (i % 100000 == 0)
        qDebug() << i << data[i];
    }
}

// Returns the DataPoint with the control z.
// If no point was found, the id of the returned control point is 0.
DataPoint LLKR::findDataPoint(NVec<3> z)
{
    DataPoint dp;
    dp.id = 0;
    for (int i = 0; i < data.size(); i++)
        if ((data[i].z - z).norm() < EPSILON)
            return data[i];
    return dp;
}

// Sets the Jacobian inverse and the error rate for the data point.
void LLKR::setKernelData(uint id, NMat<3,3> Jinv, NVec<3> error)
{
    data[id].Jinv = Jinv;
    data[id].errorRate = error;
}

// Computes the Jacobian and the error rate for the kernel identified by id (and its symmetrical pair).
void LLKR::computeKernelData(uint id)
{
    data[id].computeKernelData();
    setKernelData(id+sgn(data[id].z.z), -data[id].Jinv, data[id].errorRate);
}

// Returns true if the kernel for the data point with the id is known.
bool LLKR::isKernelKnown(uint id)
{
    return data[id].isKnown();
}

// Returns the number of points in the data set.
int LLKR::getLoadedPointCount()
{
    return data.size();
}

// Retuns the number of known transform matrices.
int LLKR::getKnownKernelCount()
{
    int count = 0;
    for (int i = 0; i < data.size(); i++)
        if (data[i].isKnown())
            count++;
    return count;
}

// Saves the LKR in a binary file.
void LLKR::save(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "LKR::save(): invalid file name" << name;
        return;
    }
    fileName += ".lkr";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "LKR::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << data;
    file.close();
}

// Loads a binary saved LKR.
void LLKR::load(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "LKR::load(): invalid file name" << name;
        return;
    }
    fileName += ".lkr";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "LKR::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> data;
    file.close();
    init();
}

// Adds a single point to the data point set.
void LLKR::addDataPoint(DataPoint p)
{
    QMutexLocker locker(&mutex);
    p.id = data.size();
    data << p;
}

// Adds a single point to the data point set.
void LLKR::addDataPoint(const NVec<3> x, const NVec<3> z)
{
    QMutexLocker locker(&mutex);
    DataPoint p;
    p.id = data.size();
    p.x = x;
    p.z = z;
    data << p;
}

// Computes a linear transformation matrix based on the eigendecomposition
// of the covariance matrix of the loaded data points in X.
NMat<3,3> LLKR::computePrincipalTransform()
{
    // Compute the covariance matrix assuming 0 mean!.
    arma::Mat<double> C(3,3);
    C.zeros();
    for (int i=0; i<3; i++)
    {
        for (int j=i; j<3; j++)
        {
            for (int k = 0; k < data.size(); k++)
                C(i,j) += (data[k].x[i])*(data[k].x[j]);
            C(i,j) /= (data.size()-1); // Note the -1!
            C(j,i) = C(i,j);
        }
    }
    //C.print("Lin C:");

    // Eigendecomposition.
    arma::Col<double> eigval(3);
    arma::Mat<double> eigvec(3,3);
    eig_sym(eigval, eigvec, C, "std");
    eigvec = fliplr(eigvec);
    eigval = flipud(eigval);

    //eigval.print("Lin eig val:");
    //eigvec.print("Lin eig vec:");

    // Compute the 3D transform from the eigen vectors and eigen values.
    arma::Mat<double> L(3,3);
    L.zeros();
    L(0,0) = sqrt(eigval(0));
    L(1,1) = sqrt(eigval(1));
    L(2,2) = sqrt(eigval(2));
    //L.print("L:");
    arma::Mat<double> T_(3,3);
    T_ = eigvec*L;
    //T_.print("Lin T:");

    NMat<3,3> M(T_.memptr());
    return M;
}

// Computes a linear transformation matrix based on the extreme points of the iso contour.
NMat<3,3> LLKR::computeIsoTransform(double t)
{
    DataPoint cdp;
    NVec<3> x1 = cdp.convertZtoX(1.0, 0, t);
    NVec<3> x2 = cdp.convertZtoX(0.5, 1.0, t);
    NVec<3> x3 = cdp.convertZtoX(0.5, 0.5, t);

    NMat<3,3> X;
    X(0,0) = x1.x; X(0,1) = x2.x; X(0,2) = x3.x;
    X(1,0) = x1.y; X(1,1) = x2.y; X(1,2) = x3.y;
    X(2,0) = x1.z; X(2,1) = x2.z; X(2,2) = x3.z;

    return X;
}

// Retrieves the k nearest neighbours of x in state space X
// from the data set using the kernel metrics.
// Set k to indicate how many neighbours you want.
// The way it works is that it first retrieves a Mah ball
// around the query point from the kd-tree, and then it does a
// linear search in the ball using the metric of the kernels.
const QList<DataPoint> LLKR::metricSearchX(NVec<3> x, int k)
{
    // Retrieve the Mah neighbours in a ball.
    //QList<DataPoint> radiusnn = radiusSearchX(x, config.preselectionBallRadius);
    QList<DataPoint> radiusnn = knnSearchX(x, 2000);
    if (radiusnn.size() < k)
        qDebug() << "LLKR::metricSearchX(): Not enough Mahalanobis ball KNN retrieved!";

    int kernelsComputed = 0;
    QList<DataPoint> knn;
    for (int n = 0; n < radiusnn.size(); n++)
    {
        DataPoint dp = radiusnn[n];

        if (!dp.isKnown())
        {
            computeKernelData(dp.id);
            dp = data[dp.id];
            kernelsComputed++;
        }

        dp.predict(x);
        //qDebug() << n << dp;
        if (dp.outOfBounds)
            continue;

        bool pushed = false;
        int j = 0;
        while (!pushed && j < knn.size())
        {
            if (dp.d < knn[j].d)
            {
                knn.insert(j, dp);
                pushed = true;
            }
            j++;
        }
        if (!pushed && knn.size() < k)
            knn << dp;
        if (knn.size() > k)
            knn.removeLast();
    }

    if (kernelsComputed > 0)
        qDebug() << "LLKR::metricSearchX():" << kernelsComputed << "kernels computed.";

    return knn;
}

// Retrieves the k nearest neighbours of x in state space X
// from the data set using the kernel metrics.
// Set k to indicate how many neighbours you want.
// It does a linear search using the metric of the kernels.
const QList<DataPoint> LLKR::linearMetricSearchX(NVec<3> x, int k)
{
    int kernelsComputed = 0;
    QList<DataPoint> knn;
    for (int n = 0; n < data.size(); n++)
    {
        DataPoint dp = data[n];

        if (!dp.isKnown())
        {
            computeKernelData(dp.id);
            dp = data[dp.id];
            kernelsComputed++;
        }

        dp.predict(x);
        if (dp.outOfBounds)
            continue;

        bool pushed = false;
        int j = 0;
        while (!pushed && j < knn.size())
        {
            if (dp.d < knn[j].d)
            {
                knn.insert(j, dp);
                pushed = true;
            }
            j++;
        }
        if (!pushed && knn.size() < k)
            knn << dp;
        if (knn.size() > k)
            knn.removeLast();
    }

    if (kernelsComputed > 0)
        qDebug() << "LLKR::linearMetricSearchX():" << kernelsComputed << "kernels computed.";

    return knn;
}

// Retrieves the k nearest neighbours of x from state space X.
// The Euklidean norm is used as metric used to compute distances.
// Set k to indicate how many neighbours you want.
// Set epsilon > 0 to allow an error margin that speeds up the search.
// You must add data points and call init() before calling this method.
const QList<DataPoint> LLKR::knnSearchX(NVec<3> x, int k, double epsilon)
{
    QList<DataPoint> knn;
    if (!initialized)
        return knn;

    // Untansform x.
    x = Tinv*x;

    // Prepare the query point.
    ANNpoint queryPt = annAllocPt(3); // allocate query point
    for (int d = 0; d < 3; d++)
        queryPt[d] = x[d];

    // Allocate the return structures.
    ANNidxArray nnIdx = new ANNidx[k]; // near neigh indices
    ANNdistArray dists = new ANNdist[k]; // near neighbor dists

    kdTreeX->annkSearch(
                    queryPt,	// query point
                    k,          // number of near neighbors
                    nnIdx,      // nearest neighbors indices (returned)
                    dists,      // squared distances (returned)
                    epsilon);   // error bound

    // Attach the distance to the data points and collect them in a list.
    for (int i=0; i<k; i++)
    {
        data[nnIdx[i]].d = sqrt(dists[i]);
        knn << data[nnIdx[i]];
    }

    delete nnIdx;
    delete dists;

    return knn;
}


// Retrieves the k nearest neighbours of y from control space Y.
// The Euklidean norm is used as metric used to compute distances.
// Set k to indicate how many neighbours you want.
// Set epsilon > 0 to allow an error margin that speeds up the search.
// You must add data points and call init() before calling this method.
const QList<DataPoint> LLKR::knnSearchZ(NVec<3> y, int k, double epsilon)
{
    QList<DataPoint> knn;
    if (!initialized)
        return knn;

    // Prepare the query point.
    ANNpoint queryPt = annAllocPt(3); // allocate query point
    for (int d = 0; d < 3; d++)
        queryPt[d] = y[d];

    // Allocate the return structures.
    ANNidxArray nnIdx = new ANNidx[k]; // near neigh indices
    ANNdistArray dists = new ANNdist[k]; // near neighbor dists

    kdTreeZ->annkSearch(
                    queryPt,	// query point
                    k,          // number of near neighbors
                    nnIdx,      // nearest neighbors indices (returned)
                    dists,      // squared distances (returned)
                    epsilon);   // error bound

    // Attach the distance to the data points and collect them in a list.
    for (int i=0; i<k; i++)
    {
        data[nnIdx[i]].d = sqrt(dists[i]);
        knn << data[nnIdx[i]];
    }

    delete nnIdx;
    delete dists;

    return knn;
}

// Retrieves the k nearest neighbours of x in state space X that are within a radius r.
// Set k=0 to retrieve ALL neighbours within the radius r.
// The Euklidean norm is used as metric to compute distances.
// Set epsilon > 0 to allow an error margin that speeds up the search.
// You must add data points and call init() before calling this method.
const QList<DataPoint> LLKR::radiusSearchX(NVec<3> x, double r, int k, double epsilon)
{
    QList<DataPoint> knn;

    if (!initialized)
        return knn;

    // Untansform x.
    x = Tinv*x;

    // Prepare the query point.
    ANNpoint queryPt = annAllocPt(3); // allocate query point
    for (int d = 0; d < 3; d++)
        queryPt[d] = x[d];

    // If k = 0, all points in the radius are retrieved.
    if (k==0)
        k = kdTreeX->annkFRSearch(queryPt, r*r, 0);

    // Allocate the return structures.
    ANNidxArray nnIdx = new ANNidx[k]; // near neigh indices
    ANNdistArray dists = new ANNdist[k]; // near neighbor dists

    kdTreeX->annkFRSearch(queryPt, // query point ANNdist
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

        data[nnIdx[i]].d = sqrt(dists[i]);
        knn << data[nnIdx[i]];
    }

    delete nnIdx;
    delete dists;

    return knn;
}


// Retrieves the k nearest neighbours of y in control space Y that are within a radius r.
// Set k=0 to retrieve ALL neighbours within the radius r.
// The Euklidean norm is used as metric to compute distances.
// Set epsilon > 0 to allow an error margin that speeds up the search.
// You must add data points and call init() before calling this method.
const QList<DataPoint> LLKR::radiusSearchZ(NVec<3> y, double r, int k, double epsilon)
{
    QList<DataPoint> knn;

    if (!initialized)
        return knn;

    // Prepare the query point.
    ANNpoint queryPt = annAllocPt(3); // allocate query point
    for (int d = 0; d < 3; d++)
        queryPt[d] = y[d];

    // If k = 0, all points in the radius are retrieved.
    if (k==0)
        k = kdTreeZ->annkFRSearch(queryPt, r*r, 0);

    // Allocate the return structures.
    ANNidxArray nnIdx = new ANNidx[k]; // near neigh indices
    ANNdistArray dists = new ANNdist[k]; // near neighbor dists

    kdTreeZ->annkFRSearch(queryPt, // query point ANNdist
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

        data[nnIdx[i]].d = sqrt(dists[i]);
        knn << data[nnIdx[i]];
    }

    delete nnIdx;
    delete dists;

    return knn;
}

// Returns the loaded point at index i.
DataPoint& LLKR::operator[](int i)
{
    return data[i];
}

// Draws points and transforms.
void LLKR::drawX(uint sampleFactor, bool showData, bool showKernels)
{
    sampleFactor = qMax((uint)1, sampleFactor);
    QMutexLocker locker(&mutex);
    for (int i=0; i < data.size(); i=i+sampleFactor)
        data[i].draw(showData, showKernels);
}

// OpenGL drawing code that draws the location of the stored data points in the abT (Z) space.
void LLKR::drawZ(uint sampleFactor)
{
    QColor color;
    double colorMin = 0;
    double colorMax = config.colorContrast;

    glPushMatrix();
    glPointSize(3);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    sampleFactor = qMax((uint)1, sampleFactor);
    for (int i=0; i < data.size(); i=i+sampleFactor)
    {
        color = colorUtil.mapColor(data[i].z.z, colorMin, colorMax);
        glColor3f(color.redF(), color.greenF(), color.blueF());
        glVertex3dv(data[i].z);
    }
    glEnd();
    glPopMatrix();
}
