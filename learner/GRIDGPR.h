#ifndef GRIDGPR_H_
#define GRIDGPR_H_
#include <globals.h>
#include <QFile>
#include <QDataStream>
#include <QList>
#include <util/NVec.h>
#include <math.h>
#include <QDebug>
#include <armadillo>
#include "GL/gl.h"
#include "GRID.h"
#include "GPR.h"
using namespace arma;

// This is a grid gpr class.

// This is a Gaussian Process Regression (GPR) class.

// To use the GPR, instantiate it and provide the amount of input dimensions (DIM) as a template
// parameter. Then provide the min and max boundaries of the input domain using the setMinMax()
// function. Then use the addPoint(X, y) function to add a data points to the training set, or the
// addPoints() method to batch add multiple data points at once. Batch adding multiple points
// is a lot faster than adding the points one by one. Use one of the evaluate*() functions to
// query the GPR for an estimation and uncertainty at any point x in the input domain.
//
// Usage example:
//
// Load data into the GP:
// GPR<2> gpr;
// gpr.setMinMax(min, max);
// foreach (frame, Data)
// {
//  NVec<2> v;
//	v[0] = frame.x;
//	v[1] = frame.y;
//	gpr.addPoint(v, frame.z);
// }
//
// And now evaluate the GP at point p:
// NVec<N> p;
// p.randomize();
// double m, s;
// evaluate(p, m, s);
// m and s now contain the mean and the stddev at point p.
//
// If you also want to draw the GPR in OpenGL, then call:
// gpr.rasterize();
// gpr.draw();
//
// You should know that the most expensive operation is adding new points to the training set.
// Evaluating the GPR is much faster, but the evaluation speed grows linearly with the amount of data.
// Rasterizing is a technique that can speed up recall at the cost of an expensive precomputation phase,
// and it is necessary for visualization.
//
// Internally, the GPR computes a covariance matrix from the given data and inverts it. The data in the
// covariance matrix is kept in a scaled form, so that the cov function returns a true circular distance.
// The raster is also kept in a scaled form.

template <int DIM=3>
class GPR
{
public:

	double p1; // max covariance parameter
	double p2; // "width" of the covariance function
	double p3; // shock when x1 == x2

	bool showStddev; // Just for drawing, show the stddev function or not.

    NVec<DIM> max; // Input range max values.
    NVec<DIM> min; // Input range min values.

	GPR();
    ~GPR(){}

	void clear();
    void setMinMax(const NVec<DIM> &minn, const NVec<DIM> &maxx);
    void setN(int N_);
	void save(QString name = "GPR");
	void load(QString name = "GPR");
	int size();
    void addPoint(const NVec<DIM> &x, const double &y);
    void addPoints(QList< NVec<DIM> > x, QList<double> y);
    double evaluateAt(const NVec<DIM> &x, double &m, double &s);
    double evaluateAt(const NVec<DIM> &x, double &m);
    double evaluateAt(const NVec<DIM> &x);
	void rasterize(); // Calculates a hyperrectangular grid of mean and stddev values.
    void drawGridSlice(double z=0, int sampleFactor=1, double zScale=1.0, double colorContrast=1.0, bool showData=false);

private:
    int N = 51; // The size of the raster in each dimension. Used for the forget bins and drawing.

    QList< NVec<DIM> > X; // The input vectors of the training data.
	Col<double> Y; // The output values of the training data (observations).
	Mat<double> K; // Covariance matrix.
	Mat<double> Kinv; // Inverse of the covariance matrix.
	Col<double> KinvY; // Cached Kinv * Y column vector for faster evaluation.

    GRID<DIM> meanGrid; // Rasterized mean values.
    GRID<DIM> stddevGrid; // Rasterized stddev values.

    double cov(const NVec<DIM> &x1, const NVec<DIM> &x2); // The data covariance function.
};

template <int DIM>
GPR<DIM>::GPR()
{
	p1 = 1.0;
	p2 = 0.01;
	p3 = 0.1;

	maxStddev = 0;
	showStddev = false;
}

// Define the covariance function here.
// Caveat: the cov function expects the data to be normalized.
template <int DIM>
double GPR<DIM>::cov(const NVec<DIM> &x1, const NVec<DIM> &x2)
{
	return p1*exp(-(x1-x2).norm2()/p2) + p3*(x1 == x2);
}

// Sets the min and max boundaries of the DIM dimensional input domain.
// These boundaries should be provided first! Setting the boundaries
// clears the data.
template <int DIM>
void GPR<DIM>::setMinMax(const NVec<DIM> &minn, const NVec<DIM> &maxx)
{
	min = minn;
	max = maxx;
	clear();
//	rasterize();
}

// Sets N, the number of nodes per dimension. This method sets N to be the same
// for every dimension and thus creates a uniform grid.
template <int DIM>
void GPR<DIM>::setN(int N_)
{
    this->N = N_;
    //rasterize();
}

// Adds a single point to the training set.
template <int DIM>
void GPR<DIM>::addPoint(const NVec<DIM> &x, const double &y)
{
	// Scale the point to the [0,1] range.
    NVec<DIM> xx = (x-min)/(max-min);

	// Discard nan cases.
    if (xx != xx || y!= y)
		return;

	// Add the point to the training set.
	int idx = X.indexOf(xx);
	if (idx >= 0)
	{
		Y(idx) = 0.5*(Y(idx) + y);
	}
	else
	{
		X << xx;
		Y.resize(Y.n_elem+1);
        Y(Y.n_elem-1) = y;
	}

	// Calculate the missing covariances.
	K.resize(X.size(), X.size());
	for (int i = 0; i < X.size(); i++)
	{
		K(X.size()-1, i) = cov(X[i], xx);
		K(i, X.size()-1) = K(X.size()-1, i);
	}

	// Update K inverse.
	Kinv = inv(sympd(K)); // Matrix inversion with symmetric and positive definite flag.
	KinvY = Kinv * Y; // Cache Kinv * Y.
}

// Batch adds a list of data to the GPR. Batch adding is much faster than adding one by one.
template <int DIM>
void GPR<DIM>::addPoints(QList< NVec<DIM> > x, QList<double> y)
{
	int batchSize = x.size();

	// Add the new points to the training set.
    NVec<DIM> xx;
	for (int k = 0; k < batchSize; k++)
	{
		// Scale the point to the [0,1] range.
        xx = (x[k]-min)/(max-min);

		// Discard NaN cases.
		if (xx != xx or y[k] != y[k])
			return;

        // If the GP already contains x, then we cannot add another x at the same place.
        // We have to average the y values.
		int idx = X.indexOf(xx);
		if (idx >= 0)
		{
			Y[idx] = 0.5*(Y[idx] + y[k]);
		}
		else
		{
			X << xx;
			Y.resize(X.size());
			Y[X.size()-1] = y[k];

			// Calculate the missing covariances.
			K.resize(X.size(), X.size());
			for (int i = 0; i < X.size(); i++)
			{
				K(X.size()-1, i) = cov(X[i], xx);
				K(i, X.size()-1) = K(X.size()-1, i);
			}
		}
	}

    // Update K inverse.
    Kinv = inv(sympd(K)); // Matrix inversion with symmetric and positive definite flag.
    KinvY = Kinv*Y; // Cache Kinv*Y.
}

// Clears the GPR and prepares for new data input.
template <int DIM>
void GPR<DIM>::clear()
{
	X.clear();
	Y.clear();

	K.resize(X.size(), X.size());
	Kinv.resize(X.size(), X.size());
    KinvY = Kinv*Y;
}

// Returns the number of points in the GP.
template <int DIM>
int GPR<DIM>::size()
{
	return Kinv.n_cols;
}

// Saves the entire GPR including the data points.
template <int DIM>
void GPR<DIM>::save(QString name)
{
    QFile file(name);
	file.open(QIODevice::WriteOnly);
	QDataStream out(&file);

	out << p1;
	out << p2;
	out << p3;
	out << size();

    out << min;
    out << max;

	for (int i = 0; i < X.size(); i++)
	{
        out << X[i];
		out << Y[i];
	}

	for (int i = 0; i < size(); i++)
		for (int j = 0; j < size(); j++)
			out << Kinv(i,j);
	for (int i = 0; i < size(); i++)
			out << KinvY[i];

	file.close();
}

// Loads a fully saved GPR.
template <int DIM>
void GPR<DIM>::load(QString name)
{
	clear();

    QFile file(name);
	if (!file.open(QIODevice::ReadOnly))
		qDebug() << "Could not open file" << file.fileName();
	QDataStream in(&file);

	int size;
    NVec<DIM> x;
	double y;

	in >> p1;
	in >> p2;
	in >> p3;

	in >> size;
    in >> min;
    in >> max;

	Y.resize(size);
	for (int i = 0; i < size; i++)
	{
        in >> x;
		in >> y;

		X << x;
		Y[i] = y;
	}

	Kinv.resize(size, size);
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			in >> Kinv(i,j);

	KinvY.resize(size);
	for (int i = 0; i < size; i++)
			in >> KinvY(i);

	file.close();
}

// Evaluates the GP at point p.
// After this method is called, m contains the mean (same as the return value) and
// s contains the stddev at the query point.
template <int DIM>
double GPR<DIM>::evaluateAt(const NVec<DIM> &p, double &m, double &s)
{
	// Scale the point to the [0,1] range.
    NVec<DIM> pp = (p-min)/(max-min);

	Row<double> Kstar(size());
	for (int i = 0; i < size(); i++)
		Kstar(i) = cov(X[i], pp);

    m = as_scalar(Kstar*KinvY);
    s = as_scalar(p1+p3 - Kstar*Kinv*Kstar.t());
	return m;
}

// Evaluates the GP at point p.
// After this method is called, m contains the mean (same as the return value).
// This version is much faster since it does not calculate the stddev.
template <int DIM>
double GPR<DIM>::evaluateAt(const NVec<DIM> &p, double &m)
{
	// Scale the point to the [0,1] range.
    NVec<DIM> pp = (p-min)/(max-min);

	Row<double> Kstar(X.size());
	for (int i = 0; i < X.size(); i++)
		Kstar(i) = cov(X[i], pp);

    m = as_scalar(Kstar*KinvY);
	return m;
}

// Evaluates the GP at point p.
// It returns the mean at point p. The stddev is not calculated.
template <int DIM>
double GPR<DIM>::evaluateAt(const NVec<DIM> &p)
{
	// Scale the point to the [0,1] range.
    NVec<DIM> pp = (p-min)/(max-min);

	Row<double> Kstar(X.size());
	for (int i = 0; i < X.size(); i++)
		Kstar(i) = cov(X[i], pp);

    return as_scalar(Kstar*KinvY);
}

// Calculates a DIM imensional hyperrectangular raster (grid) of values evenly distributed
// in the DIM dimensional [0,1] space. The bounds of the data range must be known and all
// data points must be scaled. Each dimension is divided into equal (dim) amount of nodes
// such that the first node is located at 0 and the last node is located at 1.
// The grid can then be used for drawing (and fast approximation) of the function values.
template <int DIM>
void GPR<DIM>::rasterize()
{
    meanGrid.setMinMax(min, max);
    meanGrid.setN(N);
    meanGrid.rasterize();

    stddevGrid.setMinMax(min, max);
    stddevGrid.setN(N);
    stddevGrid.rasterize();

    int nodeCount = meanGrid.getNodeCount();
    double mean = 0;
    double stddev = 0;
    QList<double> means;
    QList<double> stddevs;
    for (int n = 0; n < nodeCount; n++)
    {
        NVec<DIM> x = meanGrid.getNodeCoordinates(n);
        evaluateAt(x, mean, stddev);
        means << mean;
        stddevs << stddev;

    }
    meanGrid.setNodeOutputs(means);
    stddevGrid.setNodeOutputs(stddevs);
}


// OpenGL drawing code that draws a 2D slice of the grid.
// No interpolation is used, just the output values at the grid coordinates.
// The input parameter should be in [0,1]. It will be mapped to the "vertical"
// index of the grid slice.
template <int DIM>
void GPR<DIM>::drawGridSlice(double z, int sampleFactor, double zScale, double colorContrast, bool showData)
{
    // Compute "vertical" slice index.
    int k = qBound(0, int(z*N-1), N-1);
    sampleFactor = qMax(sampleFactor, 1);

    NVec<DIM> v;
    for (int d = 0; d < DIM; d++)
        v[d] = raster[d][idx[d]];
    return v;

    // Precompute the color table.
    double transparency = 0.95;
    double colorminz = 0;
    double colormaxz = colorContrast*max.z;
    QColor colorTable[N][N];
    for (int j = 0; j < N; j++)
    {
        for (int i = 0; i < N; i++)
        {
            int n = i + j*N + k*N*N;
            colorTable[i][j] = colorUtil.mapColor(meanGrid[n], colorminz, colormaxz);
        }
    }

    GLlib::drawWireBox(max.x, max.y, max.z);

    glPushMatrix();
    int idx[3] = {0,0,k};
    glTranslated(0, 0, meanGrid.getNodeCoordinates(idx).z);

    // Draw the grid slice.
    if (showData)
    {
        glLineWidth(1);
        glBegin( GL_LINES );
        glColor3f(0.5, 0.5, 0.5);
        for (int i = 0; i < N[0]; i=i+sampleFactor)
        {
            glVertex3f(raster[0][i], min.y, 0.0);
            glVertex3f(raster[0][i], max.y, 0.0);
        }
        for (int j = 0; j < N[1]; j=j+sampleFactor)
        {
            glVertex3f(min.x, raster[1][j], 0.0);
            glVertex3f(max.x, raster[1][j], 0.0);
        }
        glEnd();
    }
    else
    {
        glBegin(GL_QUADS);
        for (int j = 0; j < N[1]-1; j++)
        {
            for (int i = 0; i < N[0]-1; i++)
            {
                int n = i + j*cumN[1] + k*cumN[2];

                glColor4f(colorTable[i][j].redF(), colorTable[i][j].greenF(), colorTable[i][j].blueF(), transparency);
                glVertex3f(raster[0][i], raster[1][j], 0);

                glColor4f(colorTable[i+1][j].redF(), colorTable[i+1][j].greenF(), colorTable[i+1][j].blueF(), transparency);
                glVertex3f(raster[0][i+1], raster[1][j], 0);

                glColor4f(colorTable[i+1][j+1].redF(), colorTable[i+1][j+1].greenF(), colorTable[i+1][j+1].blueF(), transparency);
                glVertex3f(raster[0][i+1], raster[1][j+1], 0);

                glColor4f(colorTable[i][j+1].redF(), colorTable[i][j+1].greenF(), colorTable[i][j+1].blueF(), transparency);
                glVertex3f(raster[0][i], raster[1][j+1], 0);
            }
        }
        glEnd();
    }

    if (showData)
    {
        glPushMatrix();
        glScaled(1.0, 1.0, zScale);
        glBegin( GL_QUADS );
        for (int j = 0; j < N[1]-1; j++)
        {
            for (int i = 0; i < N[0]-1; i++)
            {
                int n = i + j*cumN[1] + k*cumN[2];

                glColor4f(colorTable[i][j].redF(), colorTable[i][j].greenF(), colorTable[i][j].blueF(), transparency);
                glVertex3f(raster[0][i], raster[1][j], Y[n]);

                glColor4f(colorTable[i+1][j].redF(), colorTable[i+1][j].greenF(), colorTable[i+1][j].blueF(), transparency);
                glVertex3f(raster[0][i+1], raster[1][j], Y[n+1]);

                glColor4f(colorTable[i+1][j+1].redF(), colorTable[i+1][j+1].greenF(), colorTable[i+1][j+1].blueF(), transparency);
                glVertex3f(raster[0][i+1], raster[1][j+1], Y[n+N[0]+1]);

                glColor4f(colorTable[i][j+1].redF(), colorTable[i][j+1].greenF(), colorTable[i][j+1].blueF(), transparency);
                glVertex3f(raster[0][i], raster[1][j+1], Y[n+N[0]]);
            }
        }
        glEnd();
        glPopMatrix();
    }

    glPopMatrix();
}

#endif /* GPR_H_ */
