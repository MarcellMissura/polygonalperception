#ifndef DATASET_H_
#define DATASET_H_
#include <util/NVec.h>
#include "GL/gl.h"
#include "util/GLlib.h"
#include "util/Statistics.h"
#include <QFile>
#include <flann/flann.hpp>
#include <QGLViewer/vec.h>
#include <QGLViewer/quaternion.h>
#include <QGLViewer/frame.h>
using namespace qglviewer;
#include "armadillo"
using namespace arma;

// The DataSet represents point cloud data, computes statistical
// values (covariance matrix), Mahalanobis distance, and draws.

template <uint DIM=3>
class DataSet
{

protected:

    QVector<double> data;
    NVec<DIM> min;
    NVec<DIM> max;
    arma::Col<double> mean;

    arma::Mat<double> c;
    arma::Mat<double> cinv;
    arma::Col<double> eigval;
    arma::Mat<double> eigvec;

public:

    DataSet();
    DataSet(int size);
    ~DataSet(){}

    void reset(); // Resets to a blank state.
    void clear(); // Resets to a blank state.
    void setSize(int size); // Use this to set the size of the set.
    void init(); // Call this after all data has been loaded.

    virtual void load(QString name = "DataSet.dat");
    virtual void save(QString name = "DataSet.dat");

    int getSize();
    void addDataPoint(const NVec<DIM> x);
    const QList<NVec<DIM> > retrieveNN(const NVec<DIM> x, int k=1);
    NVec<DIM> getMean();
    NVec<DIM> getMin();
    NVec<DIM> getMax();

    double mhd(const NVec<DIM> x);
    double mhd(const NVec<DIM> x, const NVec<DIM> y);

    // OpenGL drawing code.
    void draw(uint sampleFactor=1);
};

// Boring constructor.
template <uint DIM>
DataSet<DIM>::DataSet()
{
    c.set_size(DIM, DIM);
    cinv.set_size(DIM, DIM);
    mean.set_size(DIM);
}

// Constructor with known size of the dataset.
// It speeds up things and improves memory consumption.
template <uint DIM>
DataSet<DIM>::DataSet(int size)
{
    data.resize(size*DIM);
    c.set_size(DIM, DIM);
    cinv.set_size(DIM, DIM);
    mean.set_size(DIM);
}

// Call this method to indicate how many datapoints you expect.
// This may speed up point adding when memory does not have to
// reallocated in order to grow the data array on the fly.
template <uint DIM>
void DataSet<DIM>::setSize(int size)
{
    data.resize(size*DIM);
}

// Call this method to indicate how many datapoints you expect.
// This may speed up point adding when memory does not have to
// reallocated in order to grow the data array on the fly.
template <uint DIM>
int DataSet<DIM>::getSize()
{
    return data.size()/DIM;
}

// Resets the data set to a blank state.
template <uint DIM>
void DataSet<DIM>::reset()
{
    data.clear();
    c.eye();
    cinv.eye();
    mean.zeros();
    min = 0;
    max = 0;
}

// Resets the data set to a blank state.
template <uint DIM>
void DataSet<DIM>::clear()
{
    reset();
}

// Call this method after all data has been loaded.
// It (re)computes statistical values.
template <uint DIM>
void DataSet<DIM>::init()
{
    if (data.isEmpty())
        return;

    data.squeeze(); // Release superfluous memory.

    // Compute min, max, mean.
    min = NVec<DIM>(data.constData());
    max = NVec<DIM>(data.constData());
    mean.zeros();
    for (int k = 0; k < data.size(); k=k+DIM)
    {
        for (int d=0; d<DIM; d++)
        {
            min[d] = qMin(min[d], data[k+d]);
            max[d] = qMax(max[d], data[k+d]);
            mean(d) += data[k+d];
        }
    }
    mean /= getSize();

    // Compute the covariance matrix.
    c.fill(0);
    for (int i=0; i<DIM; i++)
    {
        for (int j=i; j<DIM; j++)
        {
            for (int k=0; k < data.size(); k=k+DIM)
            {
                c(i,j) += (data[k+i]-mean(i))*(data[k+j]-mean(j));
                c(j,i) = c(i,j);
            }
        }
    }
    c /= (getSize()-1);
//    c.print("C:");
    cinv = inv_sympd(c);

    // Eigen values and eigen vectors.
    eig_sym(eigval, eigvec, c);

//    eigval.print("val:");
//    eigvec.print("vec:");
}

// Computes the Mahalanobis distance of x.
template <uint DIM>
double DataSet<DIM>::mhd(const NVec<DIM> x)
{
    Col<double> xx(DIM);
    for (int d=0; d<DIM; d++)
        xx(d) = x[d];
    return sqrt(as_scalar((xx-mean).t()*cinv*(xx-mean)));
}

// Computes the Mahalanobis distance between two points x and y.
template <uint DIM>
double DataSet<DIM>::mhd(const NVec<DIM> x, const NVec<DIM> y)
{
    Col<double> xx(DIM);
    for (int d=0; d<DIM; d++)
        xx(d) = x[d];
    Col<double> yy(DIM);
    for (int d=0; d<DIM; d++)
        yy(d) = y[d];
    return sqrt(as_scalar((xx-yy).t()*cinv*(xx-yy)));
}

// Retrieves the k nearest neighbours of x from the data set.
template <uint DIM>
const QList<NVec<DIM> > DataSet<DIM>::retrieveNN(const NVec<DIM> x, int k)
{

    flann::Matrix<double> dataset(data.constData(), getSize(), DIM);
    flann::Matrix<double> query(x.data(), 1, DIM);
    flann::Matrix<int> indices(new int[query.rows*k], query.rows, k);
    flann::Matrix<double> dists(new float[query.rows*k], query.rows, k);

    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();
    // do a knn search, using 128 checks
    //index.knnSearch(query, indices, dists, k, flann::SearchParams(128));

    //delete[] dataset.ptr();
    //delete[] query.ptr();
    delete[] indices.ptr();
    delete[] dists.ptr();
}

// Loads a binary saved DataSet.
template <uint DIM>
void DataSet<DIM>::load(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "DataSet::load(): invalid file name" << name;
        return;
    }
    fileName += ".das";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "DataSet::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> data;
    in >> min;
    in >> max;
    file.close();

    init();
}

// Saves the DataSet in a binary file.
template <uint DIM>
void DataSet<DIM>::save(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "DataSet::save(): invalid file name" << name;
        return;
    }
    fileName += ".das";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "DataSet::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << data;
    out << min;
    out << max;
    file.close();
}

// Adds a single data point to the data set.
template <uint DIM>
void DataSet<DIM>::addDataPoint(const NVec<DIM> x)
{
    if (data.isEmpty())
    {
        min = x;
        max = x;
        for (int d=0; d<DIM; d++)
            mean(d) = x[d];
    }

    for (int d=0; d<DIM; d++)
    {
        min[d] = qMin(min[d], x[d]);
        max[d] = qMax(max[d], x[d]);
        mean(d) = (mean(d)*data.size()+x[d])/(data.size()+1);
    }

    for (int d=0; d<DIM; d++)
        data << x[d];
}

// Returns the mean of the data.
template <uint DIM>
NVec<DIM> DataSet<DIM>::getMean()
{
    NVec<DIM> v;
    for (int d=0; d<DIM; d++)
        v[d] = mean(d);
    return v;
}

// Returns the min of the data.
template <uint DIM>
NVec<DIM> DataSet<DIM>::getMin()
{
    return min;
}

// Returns the max of the data.
template <uint DIM>
NVec<DIM> DataSet<DIM>::getMax()
{
    return max;
}

// OpenGL drawing code that draws the location of the stored data points.
template <uint DIM>
void DataSet<DIM>::draw(uint sampleFactor)
{
    sampleFactor = qMax((uint)1, sampleFactor);

    glPointSize(4);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    for (int k=0; k < data.size(); k=k+DIM*sampleFactor)
        glVertex3d(data[k], data[k+1], data[k+2]);
    glEnd();

    // Draw the covariance ellipsis.

    // Construct the orientation from the eigenvectors.
    Vec vx(eigvec(0,2), eigvec(1,2), eigvec(2,2));
    Vec vy(eigvec(0,1), eigvec(1,1), eigvec(2,1));
    //Vec vz(eigvec(0,0), eigvec(1,0), eigvec(2,0));
    Vec vz = vx^vy; // cross product because the third eigen vector isn't always in the right direction
    Quaternion q;
    q.setFromRotatedBasis(vx, vy, vz);
    Frame frame;
    frame.setPosition(Vec(mean(0), mean(1), mean(2)));
    frame.setOrientation(q);

    // Compute the scaling factors based on a 95% confidence interval and the eigen values.
    double s = 7.815;
    double scalex = sqrt(s*eigval(2));
    double scaley = sqrt(s*eigval(1));
    double scalez = sqrt(s*eigval(0));

    glPushMatrix();
    glMultMatrixd(frame.matrix());
    glScaled(scalex, scaley, scalez);
    GLlib::drawFrame(1.0);
    glColor4f(0.5,0.5,0.5, 0.5);
    GLlib::drawSphere();
    glPopMatrix();
}

#endif /* DATASET_H_ */
