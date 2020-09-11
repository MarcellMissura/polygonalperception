#include "LWPR3D.h"
#include "util/Vec3f.h"
#include "util/Logger.h"
#include "util/GLlib.h"
#include <QFile>
#include <QString>
#include <QStringList>

// init(): init clears the lwpr model and the data bins.
// rasterize(): Min, max, N, and the raster are needed before you start adding data so that the data bins are defined.
// Drawing needs to be prepared by precomputing the function values at the grid nodes.

LWPR3D::LWPR3D()
{
    model = 0;

    kernelSize = 1;
    kernelSizeAdaptation = true;
    learningRate = 1;

    N = 51;
    min = -1.0; // Data range limit.
    max = 1.0; // Data range limit.
    maxBinSize = 20;  // The maximum number of points allowed to be in a bin. Older points are discarded. 0 means no limit.
}

// Sets the initial kernel size parameter (initD).
// Sets this up before training.
void LWPR3D::setKernelSize(double ks)
{
    kernelSize = ks;
    init();
}

// Turns the kernel size adaptation (updateD) on or off.
// It is on by default. Do this before training.
void LWPR3D::setKernelSizeAdaptation(bool ksa)
{
    kernelSizeAdaptation = ksa;
    init();
}

// Sets the initial learning rate (init alpha).
// It doesn't have a strong effect. Do this before training.
void LWPR3D::setLearningRate(double lr)
{
    learningRate = lr;
    init();
}

// Sets the min and max boundaries of the data range. Changing the data range will result in a complete
// reinitialization. All data and learning progress will be lost.
void LWPR3D::setMinMax(double minx1, double maxx1, double minx2, double maxx2, double minx3, double maxx3)
{
	min[0] = minx1;
	max[0] = maxx1;
	min[1] = minx2;
	max[1] = maxx2;
	min[2] = minx3;
	max[2] = maxx3;

    binX.clear();
    binY.clear();
    init();
    rasterize();
}

// Sets the min and max boundaries of the data range. Changing the data range will result in a complete
// reinitialization. All data and learning progress will be lost.
void LWPR3D::setMinMax(NVec<DIM> minn, NVec<DIM> maxx)
{
	min = minn;
	max = maxx;
    binX.clear();
    binY.clear();
    init();
    rasterize();
}

// Sets the bins per dimension parameter. Changing the number of bins will result in a complete
// reinitialization. All data and learning progress will be lost.
void LWPR3D::setN(int N)
{
    this->N = N;

    binX.clear();
    binY.clear();
    rasterize();
}

// Sets the maximum number of allowed data points in a bin.
// Can be changed any time, even during training.
void LWPR3D::setMaxBinSize(int maxBinSize)
{
	this->maxBinSize = maxBinSize;
}


// Initializes the learning process.
void LWPR3D::init()
{
    QMutexLocker locker(&mutex);

    binX.clear();
	binY.clear();

    if (model != 0)
        delete model;

    model = new LWPR_Object(DIM, 1);
	doubleVec norm_in(DIM);
	for (int d = 0; d < DIM; d++)
        norm_in[d] = (max[d]-min[d]);
	model->normIn(norm_in);
    model->setInitD(kernelSize);
    model->updateD(kernelSizeAdaptation);
    model->setInitAlpha(learningRate);
    //model->penalty(gamma);
}

// Computes a grid over the input space.
void LWPR3D::rasterize()
{
    // Calculate a minimal representation of the raster coordinates.
    raster.resize(N);
    NVec<DIM> stride = (max-min)/(N-1);
    for (int i = 0; i < N; i++)
        raster[i] = min + i*stride;
}

// Loads the data saved in a text file.
// There has to be exactly DIM+1 fields in every line, DIM fields for the
// input coordinates and 1 field for the output value at this location.
// The first line has to describe the min values of the data range (bottom left corner).
// The last line has to describe the max values of the data range (top right corner).
void LWPR3D::loadTxt(char *fileName)
{
    QString line;
    QStringList tokenList;
    bool ok;
    double value;
    int idx = 0;

    qDebug() << "LWPR3D loading file" << fileName;

    // Open the data file.
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Could not open" << file.fileName();
        return;
    }
    QTextStream in(&file);

    // Slurp the file up, ignore lines with incorrect number of fields.
    int linesSkipped = 0;
    QList< QList<double> > data;
    while (!in.atEnd())
    {
        line = in.readLine();
        if (line.startsWith("//") || line.startsWith('#') || line == "")
            continue;

        tokenList = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
        if (tokenList.size() != DIM+1)
        {
            //qDebug() << "skipped line:" << line;
            linesSkipped++;
            continue;
        }

        data << QList<double>();
        for (int i = 0; i < tokenList.size(); i++)
        {
            value = tokenList[i].toDouble(&ok);
            if (ok)
            {
                data.last() << value;
            }
            else
            {
                qDebug() << "Could not read token" << i << "\"" << tokenList[i] << "\" in line" << idx << "in file" << file.fileName();
                linesSkipped++;
                data.last() << 0;
            }
        }

        idx++;
    }

    file.close();

    qDebug() << "File successfully read." << data.size() << "points read," << linesSkipped << "lines skipped. Importing...";

    // Determine the min max data boundaries.
    for (int i = 0; i < data.size(); i++)
    {
        for (int d = 0; d < DIM; d++)
        {
            min[d] = qMin(data[i][d], min[d]);
            max[d] = qMax(data[i][d], max[d]);
        }
    }

    // Build the internal data structures from the data.
    rasterize();

    // Load the data into the model.
    init();
    for (int i = 0; i < data.size(); i++)
    {
        NVec<DIM> X(data[i][0], data[i][1], data[i][2]);
        addDataPoint(X, data[i].last());
    }

    qDebug() << "File loading complete.";
}


// Saves.
void LWPR3D::save(char *fileName)
{
    if (model == 0)
        return;

    model->writeBinary(fileName);

    QFile file("data/lwprdata.bin");
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);

    out << N;
    out << min;
    out << max;
    out << maxBinSize;
    out << binX;
    out << binY;
    out << Y;
    out << C;

    file.close();
}

// Loads.
void LWPR3D::load(char* fileName)
{
    binX.clear();
    binY.clear();

    if (model != 0)
        delete model;

    model = new LWPR_Object(fileName);

    QFile file("data/lwprdata.bin");
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);

    in >> N;
    in >> min;
    in >> max;
    in >> maxBinSize;
    in >> binX;
    in >> binY;
    in >> Y;
    in >> C;

    file.close();
    rasterize();
}

// Evaluates the learned function at position X and returns the interpolated y value.
double LWPR3D::evaluateAt(NVec<DIM> X)
{
    if (model == 0)
        return 0;

    doubleVec _x(DIM);
	for (int d = 0; d < DIM; d++)
		_x[d] = X[d];
	QMutexLocker locker(&mutex);
	return model->predict(_x)[0];
}

// Evaluates the learned function at position x and returns the predicted y value as well as a confidence.
double LWPR3D::evaluateAt(NVec<DIM> X, double& confidence)
{
    if (model == 0)
        return 0;

    doubleVec _x(DIM);
	doubleVec c(1);
	for (int d = 0; d < DIM; d++)
		_x[d] = X[d];
	QMutexLocker locker(&mutex);
	doubleVec o = model->predict(_x, c);
	confidence = c[0];
	return o[0];
}

// Adds a single data point with input values x and output value y to the data set.
void LWPR3D::addDataPoint(NVec<DIM> X, double y)
{
	// Discard nan cases.
    if (X != X || y != y)
		return;

	// Discard out of range points.
	for (int d = 0; d < DIM; d++)
        if (X[d] < min[d] || X[d] > max[d])
			return;

	QMutexLocker locker(&mutex);

	// Update the LWPR model.
	doubleVec _x(DIM);
	for (int d = 0; d < DIM; d++)
		_x[d] = X[d];
	doubleVec _y(1);
	_y[0] = y;
	model->update(_x, _y);

	// Append the new data point to the nearest neighbor data bin.
	// This is just for visualization.
	NVec<DIM> nn;
	for (int d = 0; d < DIM; d++)
        nn[d] = qRound((N-1) * (X[d]-min[d])/(max[d]-min[d]));
	quint64 hknn = hashKey(nn);
	QList< NVec<DIM> > *currentBinX = &(binX[hknn]);
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
}

void LWPR3D::addData(QList< NVec<DIM> > X, QList<double> Y)
{
    double mse = 0;
    for (int i = 0; i < X.size(); i++)
	{
        NVec<DIM> x = X[i];
        double y = Y[i];

		// Discard nan cases.
        if (x != x || y != y)
        {
            //qDebug() << "point" << x << y << "is NaN";
            continue;
        }

		// Discard out of range points.
        bool inRange = true;
		for (int d = 0; d < DIM; d++)
        {
            if (x[d] < min[d] || x[d] > max[d])
            {
                //qDebug() << "point" << x << y << "is out of range";
                inRange = false;
                break;
            }
        }
        if (!inRange)
            continue;

        QMutexLocker locker(&mutex);

		// Update the model.
		doubleVec _x(DIM);
		for (int d = 0; d < DIM; d++)
            _x[d] = x[d];
		doubleVec _y(1);
		_y[0] = y;
        doubleVec y_ = model->update(_x, _y);
        mse += (y-y_[0])*(y-y_[0]);

        if (i % 1000 == 0)
        {
            int rfs = model->numRFS(0);
            qDebug() << i << "RF:" << rfs << "alpha" << learningRate << "mse:" << mse/(i+1);
        }


		// Append the new data point to the nearest neighbor data bin.
		NVec<DIM> nn;
		for (int d = 0; d < DIM; d++)
            nn[d] = qRound((N-1) * (x[d]-min[d])/(max[d]-min[d]));
		quint64 hknn = hashKey(nn);
		QList< NVec<DIM> > *currentBinX = &(binX[hknn]);
		QList<double> *currentBinY = &(binY[hknn]);
		if (maxBinSize > 0)
		{
			while (currentBinX->size() > maxBinSize-1)
			{
				currentBinX->pop_front();
				currentBinY->pop_front();
			}
		}
        currentBinX->push_back(x);
		currentBinY->push_back(y);
	}
}

quint64 LWPR3D::hashKey(int i, int j, int k)
{
    return i+j*N+k*N*N;
}

quint64 LWPR3D::hashKey(NVec<DIM> x)
{
	quint64 hk = 0;
	for (int d = 0; d < DIM; d++)
        hk += qRound(x[d])*(quint64)pow(N, d);
	return hk;
}

// Returns a metric of the LWPR.
double LWPR3D::metric()
{
	double metric = 0;
    for (int i = 0; i < N; i++)
	{
        for (int j = 0; j < N; j++)
		{
            for (int k = 0; k < N; k++)
			{
				double v = evaluateAt(NVec<3>(raster[i].x, raster[j].y, raster[k].z));
				metric += v*v;
			}
		}
	}

	metric = sqrt(metric);

	return metric;
}


// Draw the receptive fields.
void LWPR3D::drawReceptiveFields()
{
    glPushMatrix();
    doubleVec n = model->normIn();
    //glScaled(n[0], n[1], n[2]);
    int rfs = model->numRFS(0);
    for (int i = 0; i < rfs; i++)
    {
        LWPR_ReceptiveFieldObject rf = model->getRF(0, i);
        doubleVec center = rf.center();
        doubleVec slope = rf.slope();
        std::vector<doubleVec> D = rf.D();

        // Scale the center from the unit space to the input space.
        center[0] = center[0]*n[0];
        center[1] = center[1]*n[1];
        center[2] = center[2]*n[2];

        Vec3f x = Vec3f(n[0], 0.0, slope[0]).normalized()/sqrt(D[0][0]);
        Vec3f y = Vec3f(0.0, n[1], slope[1]).normalized()/sqrt(D[1][1]);
        Vec3f z = Vec3f(0.0, -slope[2], n[2]).normalized()/sqrt(D[2][2]);

        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(0.6, 0.0, 0.0);
        glVertex3f(center[0], center[1], center[2]);
        glEnd();

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(center[0], center[1], center[2]);
        glVertex3f(center[0]+x.x, center[1]+x.y, center[2]+x.z);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(center[0], center[1], center[2]);
        glVertex3f(center[0]+y.x, center[1]+y.y, center[2]+y.z);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(center[0], center[1], center[2]);
        glVertex3f(center[0]+z.x, center[1]+z.y, center[2]+z.z);
        glEnd();
    }
    glPopMatrix();
}

// Pre-evaluates the learned function on the grid so that drawing can be fast.
void LWPR3D::prepareDrawing()
{
    Y.resize(N*N*N);
    C.resize(N*N*N);

    double v, c;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            for (int k = 0; k < N; k++)
            {
                v = evaluateAt(NVec<3>(raster[i].x, raster[j].y, raster[k].z), c);
                Y[i+j*N+k*N*N] = v;
                C[i+j*N+k*N*N] = c;
            }
        }
    }
}

void LWPR3D::draw(double z, int sampleFactor, double zScale, double colorContrast, double transparency, bool showData)
{
    // Compute "vertical" slice index.
    int k = qBound(0, int(z*N), N-1);
    sampleFactor = qMax(sampleFactor, 1);

    // Precompute the color table.
    //double transparency = 0.95;
    double colorminz = 0;
    double colormaxz = colorContrast*max.z;
    QColor colorTable[N][N];
    for (int j = 0; j < N; j++)
    {
        for (int i = 0; i < N; i++)
        {
            colorTable[i][j] = colorUtil.mapColor(Y[i+j*N+k*N*N], colorminz, colormaxz);
            colorTable[i][j].setAlphaF(qBound(0.0,(1.0-transparency)*(1.0-C[i+j*N+k*N*N]), 1.0));
        }
    }


    GLlib::drawWireFrame(max.x, max.y, max.z);
    //drawReceptiveFields();

    // Draw the grid slice.
    glPushMatrix();
    glTranslated(0, 0, raster[k].z);
    if (showData)
    {
        glLineWidth(1);
        glBegin(GL_LINES);
        glColor3f(0.5, 0.5, 0.5);
        for (int i = 0; i < N; i=i+sampleFactor)
        {
            glVertex3f(raster[i].x, min.y, 0.0);
            glVertex3f(raster[i].x, max.y, 0.0);
        }
        for (int j = 0; j < N; j=j+sampleFactor)
        {
            glVertex3f(min.x, raster[j].y, 0.0);
            glVertex3f(max.x, raster[j].y, 0.0);
        }
        glEnd();
    }
    else
    {
        glBegin(GL_QUADS);
        for (int j = 0; j < N-1; j++)
        {
            for (int i = 0; i < N-1; i++)
            {
                glColor4f(colorTable[i][j].redF(), colorTable[i][j].greenF(), colorTable[i][j].blueF(), colorTable[i][j].alphaF());
                glVertex3f(raster[i].x, raster[j].y, 0);

                glColor4f(colorTable[i+1][j].redF(), colorTable[i+1][j].greenF(), colorTable[i+1][j].blueF(), colorTable[i][j].alphaF());
                glVertex3f(raster[i+1].x, raster[j].y, 0);

                glColor4f(colorTable[i+1][j+1].redF(), colorTable[i+1][j+1].greenF(), colorTable[i+1][j+1].blueF(), colorTable[i][j].alphaF());
                glVertex3f(raster[i+1].x, raster[j+1].y, 0);

                glColor4f(colorTable[i][j+1].redF(), colorTable[i][j+1].greenF(), colorTable[i][j+1].blueF(), colorTable[i][j].alphaF());
                glVertex3f(raster[i].x, raster[j+1].y, 0);
            }
        }
        glEnd();
    }

    if (showData)
    {
        glPushMatrix();
        glScaled(1.0, 1.0, zScale);

        // Draw the function surface.
        for (int j = 0; j < N-1; j++)
        {
            for (int i = 0; i < N-1; i++)
            {
                if (Y[i+j*N+k*N*N] != 0 && Y[(i+1)+j*N+k*N*N] != 0 && Y[i+(j+1)*N+k*N*N] != 0 && Y[(i+1)+(j+1)*N+k*N*N])
                {
                    glBegin(GL_QUADS);
                    glColor4f(colorTable[i][j].redF(), colorTable[i][j].greenF(), colorTable[i][j].blueF(), colorTable[i][j].alphaF());
                    glVertex3f(raster[i].x, raster[j].y, Y[i+j*N+k*N*N]);
                    glColor4f(colorTable[i+1][j].redF(), colorTable[i+1][j].greenF(), colorTable[i+1][j].blueF(), colorTable[i+1][j].alphaF());
                    glVertex3f(raster[i+1].x, raster[j].y, Y[(i+1)+j*N+k*N*N]);
                    glColor4f(colorTable[i+1][j+1].redF(), colorTable[i+1][j+1].greenF(), colorTable[i+1][j+1].blueF(), colorTable[i+1][j+1].alphaF());
                    glVertex3f(raster[i+1].x, raster[j+1].y, Y[(i+1)+(j+1)*N+k*N*N]);
                    glColor4f(colorTable[i][j+1].redF(), colorTable[i][j+1].greenF(), colorTable[i][j+1].blueF(), colorTable[i][j+1].alphaF());
                    glVertex3f(raster[i].x, raster[j+1].y, Y[i+(j+1)*N+k*N*N]);
                    glEnd();

                    glLineWidth(1);
                    glBegin(GL_LINES);
                    glColor3f(colorTable[i][j].redF(), colorTable[i][j].greenF(), colorTable[i][j].blueF());
                    glVertex3f(raster[i].x, raster[j].y, Y[i+j*N+k*N*N]);
                    glColor3f(colorTable[i+1][j].redF(), colorTable[i+1][j].greenF(), colorTable[i+1][j].blueF());
                    glVertex3f(raster[i+1].x, raster[j].y, Y[(i+1)+j*N+k*N*N]);
                    glColor3f(colorTable[i][j].redF(), colorTable[i][j].greenF(), colorTable[i][j].blueF());
                    glVertex3f(raster[i].x, raster[j].y, Y[i+j*N+k*N*N]);
                    glColor3f(colorTable[i][j+1].redF(), colorTable[i][j+1].greenF(), colorTable[i][j+1].blueF());
                    glVertex3f(raster[i].x, raster[j+1].y, Y[i+(j+1)*N+k*N*N]);
                    glEnd();
                }
            }
        }

        // Draw the data points.
        QMutexLocker locker(&mutex);
        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0, 0, 0);
        for (int i = 0; i < N; i++)
		{
            for (int j = 0; j < N; j++)
			{
				quint64 hk = hashKey(i,j,k);
				if (binX.contains(hk))
				{
					QList< NVec<DIM> > currentBinX = binX[hk];
					QList<double> currentBinY = binY[hk];
					for (int l = 0; l < currentBinX.size(); l++)
						glVertex3f(currentBinX[l].x, currentBinX[l].y, currentBinY[l]);
				}
			}
		}
		glEnd();

        glPopMatrix();
	}

    glPopMatrix();
}

// Writes a  grid slice into a file for further processing (plotting).
void LWPR3D::logMap(QString fileName, double x3)
{
	// Calculate the "vertical" bin index.
    int k = qRound((N-1) * (x3 - min.z)/(max.z - min.z));

	// Precompute the evaluated values.
    double values[N][N];
    double confidences[N][N];
    for (int i = 0; i < N; i++)
	{
        for (int j = 0; j < N; j++)
		{
			double v, c;
			v = evaluateAt(NVec<3>(raster[i].x, raster[j].y, raster[k].z), c);
			values[i][j] = v;
            confidences[i][j] = qBound(0.0, 0.5*c, 1.0);
		}
	}

	// Find min max.
	double min = 0;
	double max = 0;
    for (int i = 0; i < N; i++)
	{
        for (int j = 0; j < N; j++)
		{
			min = qMin(values[i][j], min);
			max = qMax(max, values[i][j]);
		}
	}

	Logger logger(fileName);
	logger.clear();
    for (int j = 0; j < N; j++)
	{
        for (int i = 0; i < N; i++)
			logger << 0.4*values[i][j];
		logger++;
	}

	//qDebug() << "min:" << min << "max:" << max;
}

// Writes the saved data into a file.
void LWPR3D::logData(QString fileName, double x3)
{
	// Calculate the "vertical" bin index.
    int k = qRound((N-1) * (x3 - min.z)/(max.z - min.z));

	Logger logger(fileName);
	logger.clear();

    for (int i = 0; i < N; i++)
	{
        for (int j = 0; j < N; j++)
		{
			quint64 hk = hashKey(i,j,k);
			if (binX.contains(hk))
			{
				QList< NVec<DIM> > currentBinX = binX[hk];
				QList<double> currentBinY = binY[hk];
				for (int l = 0; l < currentBinX.size(); l++)
				{
					logger << currentBinX[l].x << currentBinX[l].y  << currentBinX[l].z << currentBinY[l];
					logger++;
				}
			}
		}
	}
}

// Writes debug information about the receptive fields into the console.
void LWPR3D::printReceptiveFields()
{
    int rfs = model->numRFS(0);
    for (int i = 0; i < rfs; i++)
    {
        LWPR_ReceptiveFieldObject rf = model->getRF(0, i);
        doubleVec center = rf.center();
        doubleVec slope = rf.slope();
        std::vector<doubleVec> D = rf.D();
        qDebug() << "RF" << i << "[" << center[0] << center[1] << center[2] << "]"
                 << "trust:" << rf.trustworthy()
                 << "PLS directions:" << rf.nReg()
                 << "slope: [" << slope[0] << slope[1] << slope[2] << "]"
                 << "D: [" << D[0][0] << D[1][1] << D[2][2] << "]";

    }
}
