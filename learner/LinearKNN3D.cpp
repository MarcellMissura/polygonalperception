#include "LinearKNN3D.h"
#include "globals.h"
#include "util/GLlib.h"
#include "util/ColorUtil.h"
#include "framework/Config.h"
#include "framework/State.h"
#include "pml/poc.h"
#include <armadillo>
#include <QGLViewer/vec.h>
#include <QGLViewer/quaternion.h>

// This is a linear KNN searcher designed to determine distances with
// the sqrt(xT*M*x) type of metric, where M = TinvT*Tinv and T is a
// linear transformation matrix.

// Boring constructor.
LinearKNN3D::LinearKNN3D()
{
    loadedPoints = 0;
}

// Destructor.
LinearKNN3D::~LinearKNN3D()
{

}

// Clears all data points and resets the knn search to a blank state.
void LinearKNN3D::reset()
{
    loadedPoints = 0;

    data.clear();
    mean = 0;
    T.eye();
    Tinv.eye();
}

// Returns the number of points in the data set.
int LinearKNN3D::getLoadedPointCount()
{
    return loadedPoints;
}

// Returns the mean of the loaded data points.
NVec<3> LinearKNN3D::getMean()
{
    return mean;
}

// Sets the mean. It only has an influence on the visualization.
void LinearKNN3D::setMean(NVec<3> m)
{
    mean = m;
}

// Returns the transformation matrix T.
NMat<3,3> LinearKNN3D::getTransform()
{
    return T;
}

// Sets the transformation matrix T and recomputes the inverse.
void LinearKNN3D::setTransform(const NMat<3,3> T_)
{
    T = T_;
    Tinv = T.inverse();
}

// Transforms all points with the inverse of the set transform T.
void LinearKNN3D::transformPoints()
{
    for (int i = 0; i < loadedPoints; i++)
    {
        NVec<3> x(&data[i*6]);
        x = Tinv*x;
        data[i*6+0] = x[0];
        data[i*6+1] = x[1];
        data[i*6+2] = x[2];
    }
}

// Computes a linear transformation matrix based on the eigendecomposition
// of the covariance matrix of the loaded data points.
NMat<3,3> LinearKNN3D::computeCovarianceTransform()
{
    // We need the mean of the data.
    NVec<3> mean_;
    for (int k = 0; k < loadedPoints; k++)
        for (int d=0; d<3; d++)
            mean_[d] += data[k*6+d];
    mean_ /= loadedPoints;

    // Compute the covariance matrix.
    arma::Mat<double> C(3,3);
    C.zeros();
    for (int i=0; i<3; i++)
    {
        for (int j=i; j<3; j++)
        {
            for (int k = 0; k < loadedPoints; k++)
                C(i,j) += (data[k*6+i]-mean_[i])*(data[k*6+j]-mean_[j]);
            C(i,j) /= (loadedPoints-1); // Note the -1!
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

// Computes the metric of x.
double LinearKNN3D::norm(NVec<3> x, int p)
{
    x = Tinv*x;
    if (p == 1)
        return x.norm1();
    return x.norm();
}

// Computes the distance between x and y.
double LinearKNN3D::norm(NVec<3> x, NVec<3> y, int p)
{
    return norm(x-y, p);
}

// Retrieves all data points.
const QList<NVec<6> > LinearKNN3D::getDataPoints()
{
    QList<NVec<6> > points;
    for (int n = 0; n < loadedPoints; n++)
        points << NVec<6>(&data[n*6]);
    return points;
}

// Retrieves the k nearest neighbours of x from the data set.
// Set k to indicate how many neighbours you want.
// Nearest neighbours are searched in the 3 dimensional input space.
// The metric used to compute distances is defined by the transformation matrix T
// and the parameter normP, which defines what kind of norm will be applied in the transformed space.
// The returned data points are 3+DIMOUT long and contain the input coordinates
// as well as the output. It uses a brute force linear search, which is rather slow.
const QList<NVec<7> > LinearKNN3D::knnSearch(NVec<3> x, int k, int normP)
{
    QList<NVec<7> > knn;
    for (int n = 0; n < loadedPoints; n++)
    {
        double dist = norm(x-NVec<3>(&data[n*6]), normP);

        bool pushed = false;
        int j = 0;
        while (!pushed && j < knn.size())
        {
            if (dist < knn[j].last())
            {
                knn.insert(j, NVec<7>(NVec<6>(&data[n*6]), dist));
                pushed = true;
            }
            j++;
        }
        if (!pushed && knn.size() < k)
            knn << NVec<7>(NVec<6>(&data[n*6]), dist);
        if (knn.size() > k)
            knn.removeLast();
    }

    return knn;
}


// Retrieves the points from the data set that are closest to the plane.
// Set k to indicate how many neighbours you want.
// The plane is defined by the normal n, and the coordinate at the origin.
// Nearest neighbours are searched in the 3 dimensional input space.
// The returned data points are 3+DIMOUT long and contain the input coordinates
// as well as the output. It uses a brute force linear search, which is rather slow.
const QList<NVec<7> > LinearKNN3D::planeSelect(NVec<3> x, NVec<3> normal, int k)
{
    QList<NVec<7> > knn;
    for (int n = 0; n < loadedPoints; n++)
    {
        NVec<3> xx(&data[n*6]);
        xx -= x;
        xx.projectOnVector(normal);
        double dist = xx.norm();

        bool pushed = false;
        int j = 0;
        while (!pushed && j < knn.size())
        {
            if (dist < knn[j].last())
            {
                knn.insert(j, NVec<7>(NVec<6>(&data[n*6]), dist));
                pushed = true;
            }
            j++;
        }
        if (!pushed && knn.size() < k)
            knn << NVec<7>(NVec<6>(&data[n*6]), dist);
        if (k > 0 && knn.size() > k)
            knn.removeLast();
    }

    return knn;
}


// Retrieves the points from the data set that are closest to the line v originating at x.
// Set k to indicate how many neighbours you want.
// Nearest neighbours are searched in the 3 dimensional input space.
// The returned data points are 3+DIMOUT long and contain the input coordinates
// as well as the output. It uses a brute force linear search, which is rather slow.
const QList<NVec<7> > LinearKNN3D::lineSelect(NVec<3> x, NVec<3> v, int k)
{
    QList<NVec<7> > knn;
    for (int n = 0; n < loadedPoints; n++)
    {
        NVec<3> xx(&data[n*6]);
        xx -= x;
        NVec<3> w = xx;
        w.projectOnVector(v);
        double dist = (xx-w).norm();

        bool pushed = false;
        int j = 0;
        while (!pushed && j < knn.size())
        {
            if (dist < knn[j].last())
            {
                knn.insert(j, NVec<7>(NVec<6>(&data[n*6]), dist));
                pushed = true;
            }
            j++;
        }
        if (!pushed && knn.size() < k)
            knn << NVec<7>(NVec<6>(&data[n*6]), dist);
        if (k > 0 && knn.size() > k)
            knn.removeLast();
    }

    return knn;
}


// Retrieves the points from the data set that have the smallest angle to the vector v originating at x.
// Set k to indicate how many neighbours you want.
// Nearest neighbours are searched in the 3 dimensional input space.
// The returned data points are 3+DIMOUT long and contain the input coordinates
// as well as the output. It uses a brute force linear search, which is rather slow.
const QList<NVec<7> > LinearKNN3D::angleSelect(NVec<3> x, NVec<3> v, int k)
{
    QList<NVec<7> > knn;
    for (int n = 0; n < loadedPoints; n++)
    {
        NVec<3> xx(&data[n*6]);
        NVec<3> yy(&data[n*6+3]);
        xx -= x;
        if (xx.norm() < EPSILON) continue;
        double dist = fabs(xx.angle(v));

        bool pushed = false;
        int j = 0;
        while (!pushed && j < knn.size())
        {
            if (dist < knn[j].last())
            {
                knn.insert(j, NVec<7>(NVec<6>(&data[n*6]), dist));
                pushed = true;
            }
            j++;
        }
        if (!pushed && knn.size() < k)
            knn << NVec<7>(NVec<6>(&data[n*6]), dist);
        if (k > 0 && knn.size() > k)
            knn.removeLast();
    }

    return knn;
}

// Deletes the data points from the set whose angle with the vector v is lower than thresh.
void LinearKNN3D::angleDelete(NVec<3> x, NVec<3> v, double thresh)
{
    QList<int> toDelete;
    for (int n = 0; n < loadedPoints; n++)
    {
        NVec<3> xx(&data[n*6]);
        xx -= x;
        if (xx.norm() < EPSILON || fabs(xx.angle(v)) < thresh)
            toDelete << n;
    }

    loadedPoints -= toDelete.size();
    for (int i = toDelete.size()-1; i>=0; i--)
        data.remove(toDelete[i]*6, 6);
}

// Adds a single point to the data point set.
void LinearKNN3D::addDataPoint(const NVec<6> x)
{
    for (int d = 0; d < 6; d++)
        data.push_back(x[d]);
//    for (int d = 0; d < 3; d++)
//        mean[d] = (mean[d]*loadedPoints+x[d])/(loadedPoints+1);
    loadedPoints++;
}

// OpenGL drawing code that draws the location of the stored data points.
void LinearKNN3D::draw(uint sampleFactor, bool untransform, bool showData, bool showAxes)
{
    if (showAxes)
    {
        glPushMatrix();
        if (!untransform)
        {
            glTranslated(mean[0], mean[1], mean[2]); // translate
            glMultMatrixd(T); // rotate and scale
        }

        GLlib::drawFrame(1.0);
        glPopMatrix();
    }

    if (showData)
    {
        glPushMatrix();
        if (untransform)
        {
            glMultMatrixd(Tinv); // rotate and scale
            glTranslated(-mean[0], -mean[1], -mean[2]); // translate
        }

        QColor color;
        double colorMin = 0;
        double colorMax = config.colorContrast;

        glPointSize(5);
        glColor3f(0,0,0);
        glBegin(GL_POINTS);
        sampleFactor = qMax((uint)1, sampleFactor);
        for (int k=0; k < loadedPoints; k=k+sampleFactor)
        {
            double t = fabs(data[k*6+3])+fabs(data[k*6+4])+fabs(data[k*6+5]);
            color = colorUtil.mapColor(t, colorMin, colorMax);
            glColor3f(color.redF(), color.greenF(), color.blueF());
            glVertex3d(data[k*6+0], data[k*6+1], data[k*6+2]);
        }
        glEnd();
        glPopMatrix();
    }
}
