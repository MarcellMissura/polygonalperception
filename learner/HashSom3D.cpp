#include "HashSom3D.h"
#include "globals.h"
#include "util/Vec3f.h"
#include "util/Logger.h"
#include <math.h>
#include <QGLViewer/qglviewer.h>
#include <QFile>
#include <QDebug>
#include <QThread>

HashSom3D::HashSom3D()
{
    N = 50; // Number of nodes per dimension.
	dataGain = 1.0; // How strongly the som neurons are pulled towards the data centroids.
	neighborhoodGain = 2.0; // How strongly the som neurons are pulled towards their neighborhood centroids.
	timestep = 0.25; // Simulation time step. The larger the faster, but the more likely to explode.
	maxBinSize = 10;  // The maximum number of points allowed to be in a bin. Older points are discarded. 0 means no limit.
	neighborHoodRadius = 3; // The size of the neighborhood to operate on.

	max[0] = 1.0; // Data range limit.
	min[0] = -1.0; // Data range limit.
	max[1] = 1.0; // Data range limit.
	min[1] = -1.0; // Data range limit.
	max[2] = 1.0; // Data range limit.
	min[2] = -1.0; // Data range limit.

	nothingToDo = false;
	showDataPoints = false;
	showActivity = false;
}

// Sets the data gain that determines how strongly the neurons are pulled into the data centroids.
// Can be changed any time, even during training.
void HashSom3D::setDataGain(double dataGain)
{
	this->dataGain = dataGain;
}

// Sets the neighborhood gain that determines how strongly the neurons are pulled towards the centroid of their neighbors.
// Can be changed any time, even during training.
void HashSom3D::setNeighborhoodGain(double neighborhoodGain)
{
	this->neighborhoodGain = neighborhoodGain;
}

// Sets the time step parameter. The timestep parameter determines how far the neurons are moved per training iteration.
// The higher the time step, the faster the training converges, but if the number is too high, the network will explode.
// Can be changed any time, even during training.
void HashSom3D::setTimestep(double timestep)
{
	this->timestep = timestep;
}

// Sets the maximum number of allowed data points in a bin.
// Can be changed any time, even during training.
void HashSom3D::setMaxBinSize(int maxBinSize)
{
	this->maxBinSize = maxBinSize;
}

// Sets the radius of the neighborhood where new neurons are grown and activated.
void HashSom3D::setNeighborHoodRadius(int radius)
{
	this->neighborHoodRadius = radius;
}

// Sets the min and max boundaries of the data range. Changing the data range will result in a complete
// reinitialization of the SOM. All data and learning progress will be lost.
void HashSom3D::setMinMax(double minx1, double maxx1, double minx2, double maxx2, double minx3, double maxx3)
{
	min[0] = minx1;
	max[0] = maxx1;
	min[1] = minx2;
	max[1] = maxx2;
	min[2] = minx3;
	max[2] = maxx3;

	init();
}

// Sets the min and max boundaries of the data range. Changing the data range will result in a complete
// reinitialization of the SOM. All data and learning progress will be lost.
void HashSom3D::setMinMax(NVec<DIM> minn, NVec<DIM> maxx)
{
	min = minn;
	max = maxx;
	init();
}

// Sets the bins per dimension parameter. Changing the number of bins will result in a complete
// reinitialization of the SOM. All data and learning progress will be lost.
void HashSom3D::setBins(int bins)
{
    N = bins;
	init();
}


// Clears, resets and initializes the som grid with a BINS^DIM lattice evenly distributed in
// the data range. Min and max boundaries and the number of bins per dimensions have to be provided
// beforehand with the setMinMax() and setBins() functions. The output values of the som neurons are
// initialized with 0.
void HashSom3D::init()
{
	som.clear();
	binX.clear();
	binY.clear();
	centroid.clear();
	centroidForce.clear();
	neighborHoodForce.clear();

//	som.reserve(50000);
//	neighborHoodForce.reserve(50000);

	// Calculate a minimal representation of the raster coordinates.
    raster.resize(N);
    NVec<DIM> stride = (max-min)/(N-1);
    for (int i = 0; i < N; i++)
		raster[i] = min + i * stride;
}

// Saves the SOM.
void HashSom3D::save(QString name)
{
	// Not implemented yet.
}

// Loads the SOM.
void HashSom3D::load(QString name)
{
	// Not implemented yet.
}

// Evaluates the SOM at position x and returns the interpolated y value.
double HashSom3D::evaluateAt(NVec<DIM> x)
{
	// If these bounds are in effect, the network will not extrapolate values
	// outside the grid, but return the value at the grid border.
	//x = qBound(xmin, x, xmax);
	//y = qBound(ymin, y, ymax);

	// Calculate the base neighbor index.
    int i = qBound(0, int((N-1) * (x.x - min.x)/(max.x - min.x)), N-2);
    int j = qBound(0, int((N-1) * (x.y - min.y)/(max.y - min.y)), N-2);
    int k = qBound(0, int((N-1) * (x.z - min.z)/(max.z - min.z)), N-2);

	// Evaluate the SOM using linear interpolation between the neighbor neurons.
	double factorX1 = (x.x - raster[i].x) / (raster[i+1].x - raster[i].x);
	double factorX2 = (x.y - raster[j].y) / (raster[j+1].y - raster[j].y);
	double factorX3 = (x.z - raster[k].z) / (raster[k+1].z - raster[k].z);

	double s1 = som.value(hashKey(i,j,k));
	double s2 = som.value(hashKey(i,j+1,k));
	double s3 = som.value(hashKey(i,j,k+1));
	double s4 = som.value(hashKey(i,j+1,k+1));

	double s5 = som.value(hashKey(i+1,j,k));
	double s6 = som.value(hashKey(i+1,j+1,k));
	double s7 = som.value(hashKey(i+1,j,k+1));
	double s8 = som.value(hashKey(i+1,j+1,k+1));

	double y1 = s1 + factorX1 * (s5 - s1);
	double y2 = s2 + factorX1 * (s6 - s2);
	double y3 = s3 + factorX1 * (s7 - s3);
	double y4 = s4 + factorX1 * (s8 - s4);

	double y5 = y1 + factorX2*(y2-y1);
	double y6 = y3 + factorX2*(y4-y3);
	double y = y5 + factorX3*(y6-y5);

	return y;
}

// Adds a single data point with input values x and output value y to the data set.
void HashSom3D::addPoint(NVec<DIM> X, double y)
{
	// Discard nan cases.
	if (X != X or y != y)
		return;

	// Discard out of range points.
	for (int d = 0; d < DIM; d++)
		if (X[d] < min[d] or X[d] > max[d])
			return;

	//qDebug() << "adding" << X << y;

	QMutexLocker locker(&mutex);
	Xq << X;
	Yq << y;
}

void HashSom3D::addPoints(QList< NVec<DIM> > _X, QList<double> _y)
{
//	qDebug() << "lock addpoints" << QThread::currentThreadId();
	QMutexLocker locker(&mutex);
	for (int i = 0; i < _X.size(); i++)
	{
		NVec<DIM> X = _X[i];
		double y = _y[i];

		// Discard nan cases.
		if (X != X or y != y)
			return;

		// Discard out of range points.
		for (int d = 0; d < DIM; d++)
			if (X[d] < min[d] or X[d] > max[d])
				return;

		Xq << X;
		Yq << y;
	}
//	qDebug() << "unlock addpoints" << QThread::currentThreadId();
}


quint64 HashSom3D::hashKey(int i, int j, int k)
{
    return i + j*N + k*N*N;
}

quint64 HashSom3D::hashKey(NVec<DIM> x)
{
	quint64 hk = 0;
	for (int d = 0; d < DIM; d++)
        hk += qRound(x[d]) * (quint64)pow(N, d);
	return hk;
}

// Performs a bunch of training steps.
int HashSom3D::train(int maxIterations)
{
	int i = 0;
	nothingToDo = false;
	while (!nothingToDo and i < maxIterations)
	{
		trainingStep();
		i++;
	}

	return i;
}

// Grows new neurons, handles neuron activation, calculates the forces acting on the neurons, and moves the neurons by a bit.
void HashSom3D::trainingStep()
{

	// Take the points from the input queue.
	mutex.lock();
	QList< NVec<DIM> > Xqt;
	QList<double> Yqt;
	if (Xq.size() > 0)
	{
		foreach (NVec<DIM> X, Xq)
			Xqt << X;
		foreach (double y, Yq)
			Yqt << y;
		Xq.clear();
		Yq.clear();
	}
	mutex.unlock();

	// Insert the new data points.
	drawMutex.lock();
	for (int i = 0; i < Xqt.size(); i++)
	{
		NVec<DIM> X = Xqt[i];
		double y = Yqt[i];

		// Calculate the nearest neighbor hashkey.
		NVec<DIM> nn;
		for (int d = 0; d < DIM; d++)
            nn[d] = qRound((N-1) * (X[d] - min[d])/(max[d] - min[d]));
		quint64 hknn = hashKey(nn);

		// Append the new data point to the nearest neighbor data bin.
		QList< NVec<DIM> > *currentBinX = &(binX[hknn]);
		bool newBin = (currentBinX->size() == 0);
		QList<double> *currentBinY = &(binY[hknn]);
		if (maxBinSize > 0)
		{
			while (currentBinX->size() > maxBinSize-1)
			{
				currentBinX->pop_front();
				currentBinY->pop_front();
			}
		}
		currentBinX->push_back(X);
		currentBinY->push_back(y);

		// Calculate the new centroid.
		centroid[hknn] = (centroid[hknn] * (currentBinY->size()-1) + y) / currentBinY->size();

		// Compute the neighborhood enumeration.
		QList<quint64> nnHashKeys;
		if (newBin or hknn != lastNNhk)
			nnHashKeys = enumerateNeighborHood(X, neighborHoodRadius);

		// Grow som neurons in the neighborhood if needed and initialize with the y value of the nn.
		if (newBin)
			for (int i = 0; i < nnHashKeys.size(); i++)
				if (not som.contains(nnHashKeys[i]))
					som[nnHashKeys[i]] = centroid[hknn];

		// Activate neurons in the neighborhood, if we are touching a new bin.
		if (hknn != lastNNhk)
			for (int i = 0; i < nnHashKeys.size(); i++)
				active[nnHashKeys[i]];
		lastNNhk = hknn;
	}
	drawMutex.unlock();




	// Calculate centroid and neighborhood forces.
	QList<quint64> keys = active.keys();
	for (int i = 0; i < keys.size(); i++)
	{
		if (centroid.contains(keys[i]))
			centroidForce[keys[i]] = centroid[keys[i]] - som[keys[i]]; // Because hopefully not all data bins are active.

		int count = 0;
		double force = 0;
		double value = 0;

		value = som.value(keys[i] + 1);
		if (value != 0)
		{
			force += value;
			count++;
		}

		value = som.value(keys[i] - 1);
		if (value != 0)
		{
			force += value;
			count++;
		}

        value = som.value(keys[i] + N);
		if (value != 0)
		{
			force += value;
			count++;
		}

        value = som.value(keys[i] - N);
		if (value != 0)
		{
			force += value;
			count++;
		}

        value = som.value(keys[i] + N*N);
		if (value != 0)
		{
			force += value;
			count++;
		}

        value = som.value(keys[i] - N*N);
		if (value != 0)
		{
			force += value;
			count++;
		}

		neighborHoodForce[keys[i]] = force/count - som[keys[i]];
	}

	// Update neuron states and deactivate them if the force is too small.
	nothingToDo = true;
	for (int i = 0; i < keys.size(); i++)
	{
		nothingToDo = false;
		double cf = dataGain*centroidForce.value(keys[i]);
		double nf = neighborhoodGain*neighborHoodForce[keys[i]];
		som[keys[i]] += timestep * (cf + nf);
		if ((qAbs(cf+nf)) < 0.00001)
		{
			drawMutex.lock(); // just for drawing
			active.remove(keys[i]);
			drawMutex.unlock();
		}
	}
}

// Enumerates the neighborhood of a point X.
QList<quint64> HashSom3D::enumerateNeighborHood(NVec<DIM> X, int radius)
{
	QList<quint64> hashKeys;

	// Calculate the nearest neighbor index minus radius.
	NVec<DIM> nnmr;
	for (int d = 0; d < DIM; d++)
        nnmr[d] = qRound((N-1) * (X[d] - min[d])/(max[d] - min[d])) - radius;

	// Determine the min and max boundaries to respect the grid border.
	int bins = 2*radius + 1;
	NVec<DIM> bmin;
	NVec<DIM> bmax;
	for (int d = 0; d < DIM; d++)
	{
		bmin[d] = qMax(nnmr[d], 0.0);
        bmax[d] = qMin((int)nnmr[d]+bins, N)-1;
	}

	// Enumerate the grid cells in a cube of 2*radius.
	NVec<DIM> b = bmin;
	int d = 0;
	do
	{
		hashKeys << hashKey(b);

		// Generate next grid coordinates.
		d = 0;
		while (d < DIM)
		{
			b[d]++;
			if (b[d] <= bmax[d])
				break;
			b[d] = bmin[d];
			d++;
		}
	} while (b != bmax);

	return hashKeys;
}

// Returns a metric of the som. The metric is the Euklidean norm over the som values.
double HashSom3D::metric()
{
	double metric = 0;
    for (int i = 0; i < N; i++)
	{
        for (int j = 0; j < N; j++)
		{
            for (int k = 0; k < N; k++)
			{
				quint64 hk = hashKey(i,j,k);
				if (som.contains(hk))
				{
					metric += som[hk]*som[hk];
				}
			}
		}
	}

	metric = sqrt(metric);

	return metric;
}

void HashSom3D::draw(double x3)
{
	QColor color;
	Vec3f color1(0.2, 0.2, 0.2);
	Vec3f color2(0, 0, 0);
	double transparency = 0.75;

	// Calculate the "vertical" bin index.
    int k = qRound((N-1) * (x3 - min.z)/(max.z - min.z));

	drawMutex.lock();

	// Find min max.
	double min = 0;
	double max = 1.0;
	bool inited = false;
    for (int i = 0; i < N; i++)
	{
        for (int j = 0; j < N; j++)
		{
			quint64 hk = hashKey(i,j,k);
			if (som.contains(hk))
			{
				if (not inited)
				{
					min = som[hk];
					max = som[hk];
					inited = true;
				}
				else
				{
					min = qMin(som[hk], min);
					max = qMax(max, som[hk]);
				}
			}
		}
	}


	// Quads to fill the square space between the nodes.
	glBegin( GL_QUADS );
    for (int i = 0; i < N-1; i++)
	{
        for (int j = 0; j < N-1; j++)
		{
			quint64 hk = hashKey(i,j,k);
			quint64 hk1 = hashKey(i+1,j,k);
			quint64 hk2 = hashKey(i,j+1,k);
			quint64 hk3 = hashKey(i+1,j+1,k);
			if (som.contains(hk) and som.contains(hk1) and som.contains(hk2) and som.contains(hk3))
			{
				color = colorUtil.mapColor(som[hk], min, max);
				glColor4f(color.redF(), color.greenF(), color.blueF(), transparency);
				glVertex3f(raster[i].x, raster[j].y, som[hk]);

				color = colorUtil.mapColor(som[hk1], min, max);
				glColor4f(color.redF(), color.greenF(), color.blueF(), transparency);
				glVertex3f(raster[i+1].x, raster[j].y, som[hk1]);

				color = colorUtil.mapColor(som[hk3], min, max);
				glColor4f(color.redF(), color.greenF(), color.blueF(), transparency);
				glVertex3f(raster[i+1].x, raster[j+1].y, som[hk3]);

				color = colorUtil.mapColor(som[hk2], min, max);
				glColor4f(color.redF(), color.greenF(), color.blueF(), transparency);
				glVertex3f(raster[i].x, raster[j+1].y, som[hk2]);
			}
		}
	}
	glEnd();


	// Draw the nodes.
	if (showActivity)
	{
		glPointSize(4);
		glColor3f(1.0, 1.0, 1.0);
		glBegin( GL_POINTS );
        for (int i = 0; i < N; i++)
		{
            for (int j = 0; j < N; j++)
			{
				quint64 hk = hashKey(i,j,k);
				if (som.contains(hk))
					if (active.contains(hk))
						glVertex3f(raster[i].x, raster[j].y, som[hk]);
			}
		}
		glEnd();
	}

	// Lines connecting the nodes.
	glLineWidth(2);
	glColor3f(color1.x, color1.y, color1.z);
	glBegin( GL_LINES );
    for (int i = 0; i < N-1; i++)
	{
        for (int j = 0; j < N-1; j++)
		{
			quint64 hk = hashKey(i,j,k);
			quint64 hk1 = hashKey(i+1,j,k);
			quint64 hk2 = hashKey(i,j+1,k);
			if (som.contains(hk) and som.contains(hk1))
			{
				color = colorUtil.mapColor(som[hk], min, max);
				color.darker(300);
				glColor3f(color.redF(), color.greenF(), color.blueF());
				glVertex3f(raster[i].x, raster[j].y, som[hk]);

				color = colorUtil.mapColor(som[hk1], min, max);
				color.darker(300);
				glColor3f(color.redF(), color.greenF(), color.blueF());
				glVertex3f(raster[i+1].x, raster[j].y, som[hk1]);
			}
			if (som.contains(hk) and som.contains(hk2))
			{
				color = colorUtil.mapColor(som[hk], min, max);
				color.darker(300);
				glColor3f(color.redF(), color.greenF(), color.blueF());
				glVertex3f(raster[i].x, raster[j].y, som[hk]);

				color = colorUtil.mapColor(som[hk2], min, max);
				color.darker(300);
				glColor3f(color.redF(), color.greenF(), color.blueF());
				glVertex3f(raster[i].x, raster[j+1].y, som[hk2]);
			}
		}
	}
	glEnd();


	// Draw the training data.
	if (showDataPoints)
	{
		glPointSize(5.0);
		glBegin( GL_POINTS );
		glColor3f(color2.x, color2.y, color2.z);
        for (int i = 0; i < N; i++)
		{
            for (int j = 0; j < N; j++)
			{
				quint64 hk = hashKey(i,j,k);
//				drawMutex.lock();
				if (binX.contains(hk))
				{
					QList< NVec<DIM> > currentBinX = binX[hk];
					QList<double> currentBinY = binY[hk];
					for (int l = 0; l < currentBinX.size(); l++)
						glVertex3f(currentBinX[l].x, currentBinX[l].y, currentBinY[l]);
				}
//				drawMutex.unlock();
			}
		}
		glEnd();
	}

	drawMutex.unlock();
}

void HashSom3D::printMap(QString fileName, double x3)
{
	// Calculate the "vertical" bin index.
    int k = qRound((N-1) * (x3 - min.z)/(max.z - min.z));

	Logger logger(fileName);
	logger.clear();
    for (int j = 0; j < N; j++)
	{
        for (int i = 0; i < N; i++)
		{
			quint64 hk = hashKey(i,j,k);
			if (som.contains(hk))
				logger << som[hk];
			else
				logger << 0;
		}

		logger++;
	}
}

void HashSom3D::query()
{
	qDebug() << "capacity:" << som.capacity() << "size:" << som.size() << "active:" << active.size() << "queue length:" << Xq.size();
}
