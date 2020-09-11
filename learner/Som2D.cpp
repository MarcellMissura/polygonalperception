#include "Som2D.h"
#include <math.h>
#include <QGLViewer/qglviewer.h>
#include <QFile>
#include <QDebug>

// The SOM neural network is a simple learner for 2D input dimensions and 1 output dimension.
// The network is a lattice evenly spread out in the data range. The nodes (neurons) of the lattice
// are pulled towards the centroid of the data in the vicinity, but also towards the centroid of
// their neighborhood neurons.

Som2D::Som2D()
{
	dataGain = 1.0; // How strongly the som neurons are pulled towards the data centroids.
	neighborhoodGain = 1.0; // How strongly the som neurons are pulled towards their neighborhood centroids.
	neighborhoodBlur = 0.5; // Between 0 and 1. How strong is the blur filter that is applied to the neighborhood forces.
	timestep = 0.2; // Simulation time step. The larger the faster, but the more likely to explode.
	maxBinSize = 20;  // The maximum number of points allowed to be in a bin. Older points are discarded.
	bins = 50; // Number of bins per dimension.

	maxStdDev = 0;

	xmax = 1.0; // Data range limit.
	xmin = -1.0; // Data range limit.
	ymax = 1.0; // Data range limit.
	ymin = -1.0; // Data range limit.

	// Initialize the data structures with a SOM_DIM x SOM_DIM lattice filled with all zeros.
	memset(som, 0, SOM_DIM*SOM_DIM*sizeof(Vec3f));
	memset(centroid, 0, SOM_DIM*SOM_DIM*sizeof(double));
	memset(stddev, 0, SOM_DIM*SOM_DIM*sizeof(double));
	memset(centroidForce, 0, SOM_DIM*SOM_DIM*sizeof(double));
	memset(firstOrderForce, 0, SOM_DIM*SOM_DIM*sizeof(double));
	memset(blurredForce, 0, SOM_DIM*SOM_DIM*sizeof(double));
	memset(totalForce, 0, SOM_DIM*SOM_DIM*sizeof(double));
}

// Sets the data gain that determines how strongly the neurons are pulled into the data centroids.
// Can be changed any time, even during training.
void Som2D::setDataGain(double dataGain)
{
	this->dataGain = dataGain;
}

// Sets the neighborhood gain that determines how strongly the neurons are pulled towards the centroid of their neighbors.
// Can be changed any time, even during training.
void Som2D::setNeighborhoodGain(double neighborhoodGain)
{
	this->neighborhoodGain = neighborhoodGain;
}

// Sets the blur coefficient that determines how strongly the neighborhood forces are blurred.
// Can be changed any time, even during training.
void Som2D::setNeighborhoodBlur(double neighborhoodBlur)
{
	this->neighborhoodBlur = neighborhoodBlur;
}

// Sets the time step parameter. The timestep parameter determines how far the neurons are moved per training iteration.
// The higher the time step, the faster the training converges, but if the number is too high, the network will explode.
// Can be changed any time, even during training.
void Som2D::setTimestep(double timestep)
{
	this->timestep = timestep;
}

// Sets the maximum number of allowed data points in a bin.
// Can be changed any time, even during training.
void Som2D::setMaxBinSize(int maxBinSize)
{
	this->maxBinSize = maxBinSize;
}


// Sets the min and max boundaries of the data range. Changing the data range will result in a complete
// reinitialization of the SOM. All data and learning progress will be lost.
void Som2D::setMinMax(double minx, double maxx, double miny, double maxy)
{
	xmin = minx;
	xmax = maxx;
	ymin = miny;
	ymax = maxy;
}

void Som2D::setBins(int bins)
{
	this->bins = bins;
}


// Initializes (or resets) the som grid with a SOM_DIM x SOM_DIM lattice evenly distributed in
// the data range. Min and max boundaries have to be provided beforehand with the setMinMax() function.
// The z values of the som neurons are initialized with 0. This does not clear the data bins.
void Som2D::init()
{
	// Move the som neurons in place and initialize them with 0.
	for (int i = 0; i < SOM_DIM; i++)
	{
		for (int j = 0; j < SOM_DIM; j++)
		{
			som[i][j].x = xmin + (xmax-xmin)*(double)i/(SOM_DIM-1);
			som[i][j].y = ymin + (ymax-ymin)*(double)j/(SOM_DIM-1);
			som[i][j].z = 0.0;
		}
	}
}

// Clears the data bins, but does not reset the SOM neurons.
void Som2D::clear()
{
	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			bin[i][j].clear();
	memset(centroid, 0, SOM_DIM*SOM_DIM*sizeof(double));
	memset(stddev, 0, SOM_DIM*SOM_DIM*sizeof(double));
}

// Saves the SOM lattice, but not the data.
void Som2D::save(QString name)
{
	QFile file("data/learnedmodels/" + name + ".som");
	file.open(QIODevice::WriteOnly);
	QDataStream out(&file);
//	out.setVersion(QDataStream::Qt_3_1);

	out << xmin;
	out << xmax;
	out << ymin;
	out << ymax;

	for (int i = 0; i < SOM_DIM; i++)
	{
		for (int j = 0; j < SOM_DIM; j++)
		{
			out << som[i][j].x;
			out << som[i][j].y;
			out << som[i][j].z;
		}
	}

	file.close();
}

// Loads the SOM. After loading, the data bins are empty.
void Som2D::load(QString name)
{
	// Open the file.
	QFile file("data/learnedmodels/" + name + ".som");
	if (!file.open(QIODevice::ReadOnly))
		qDebug() << "Could not open file" << file.fileName();
	QDataStream in(&file);
	///in.setVersion(QDataStream::Qt_3_1);

	// Load the meta parameters.
	in >> xmin;
	in >> xmax;
	in >> ymin;
	in >> ymax;

	// Load the som neurons from the file.
	for (int i = 0; i < SOM_DIM; i++)
	{
		for (int j = 0; j < SOM_DIM; j++)
		{
			in >> som[i][j].x;
			in >> som[i][j].y;
			in >> som[i][j].z;
		}
	}

	file.close();

	// Reset the data bins.
	clear();
}

// Adds a single point to the data set. The data range boundaries must be already known.
void Som2D::addPoint(Vec3f v)
{
	// Discard nan cases.
	if (v != v)
		return;

	// Calculate the nearest neighbor bin index.
	int i = qRound((SOM_DIM-1) * (v.x - xmin)/(xmax - xmin));
	int j = qRound((SOM_DIM-1) * (v.y - ymin)/(ymax - ymin));

	// Discard out of range points.
	if (i < 0 or i > SOM_DIM-1 or j < 0 or j > SOM_DIM-1)
		return;

	// Append to the data bin.
	if (maxBinSize > 0)
		while (bin[i][j].size() > maxBinSize-1)
			bin[i][j].pop_front();
	bin[i][j].push_back(v);

	// Calculate the new centroid.
	centroid[i][j] = 0;
	for (int k = 0; k < bin[i][j].size(); k++)
		centroid[i][j] += bin[i][j][k].z;
	centroid[i][j] /= (double)bin[i][j].size();

	// Calculate the standard deviation as well.
	stddev[i][j] = 0;
	for (int k = 0; k < bin[i][j].size(); k++)
		stddev[i][j] += (bin[i][j][k].z - centroid[i][j])*(bin[i][j][k].z - centroid[i][j]);
	stddev[i][j] = sqrt(stddev[i][j]/(double)bin[i][j].size());
	maxStdDev = qMax(maxStdDev, stddev[i][j]);
}

void Som2D::addPoint(double x, double y, double z)
{
	addPoint(Vec3f(x,y,z));
}

// Performs a bunch of training steps.
void Som2D::train(int howManySteps)
{
	for (int i = 0; i < howManySteps; i++)
		trainingStep();
}

// Calculates the forces acting on the neurons and moves the neurons by a bit.
void Som2D::trainingStep()
{
	// Calculate centroid forces.
	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			centroidForce[i][j] = bin[i][j].size() > 0 ? centroid[i][j] - som[i][j].z : 0;

	// Calculate first order neighborhood forces with explicit border case handling.
	for (int i = 1; i < SOM_DIM-1; i++)
		for (int j = 1; j < SOM_DIM-1; j++)
			firstOrderForce[i][j] = (som[i+1][j].z + som[i-1][j].z + som[i][j+1].z + som[i][j-1].z)/4.0 - som[i][j].z;

	for (int i = 1; i < SOM_DIM-1; i++)
	{
		firstOrderForce[i][0] = (som[i+1][0].z + som[i-1][0].z + som[i][1].z)/3.0 - som[i][0].z;
		firstOrderForce[i][SOM_DIM-1] = (som[i+1][SOM_DIM-1].z + som[i-1][SOM_DIM-1].z + som[i][SOM_DIM-2].z)/3.0 - som[i][SOM_DIM-1].z;
		firstOrderForce[0][i] = (som[0][i+1].z + som[0][i-1].z + som[1][i].z)/3.0 - som[0][i].z;
		firstOrderForce[SOM_DIM-1][i] = (som[SOM_DIM-1][i+1].z + som[SOM_DIM-1][i-1].z + som[SOM_DIM-2][i].z)/3.0 - som[SOM_DIM-1][i].z;
	}

	firstOrderForce[0][0] = (som[0][1].z + som[1][0].z)/2.0 - som[0][0].z;
	firstOrderForce[0][SOM_DIM-1] = (som[0][SOM_DIM-2].z + som[1][SOM_DIM-1].z)/2.0 - som[0][SOM_DIM-1].z;
	firstOrderForce[SOM_DIM-1][0] = (som[SOM_DIM-2][0].z + som[SOM_DIM-1][1].z)/2.0 - som[SOM_DIM-1][0].z;
	firstOrderForce[SOM_DIM-1][SOM_DIM-1] = (som[SOM_DIM-2][SOM_DIM-1].z + som[SOM_DIM-1][SOM_DIM-2].z)/2.0 - som[SOM_DIM-1][SOM_DIM-1].z;


	// Apply a "blur filter" on the first order forces.
	for (int i = 1; i < SOM_DIM-1; i++)
		for (int j = 1; j < SOM_DIM-1; j++)
			blurredForce[i][j] = 0.25*(firstOrderForce[i+1][j] + firstOrderForce[i-1][j] + firstOrderForce[i][j+1] + firstOrderForce[i][j-1]);

	for (int i = 1; i < SOM_DIM-1; i++)
	{
		blurredForce[i][0] = (firstOrderForce[i+1][0] + firstOrderForce[i-1][0] + firstOrderForce[i][1])/3.0;
		blurredForce[i][SOM_DIM-1] = (firstOrderForce[i+1][SOM_DIM-1] + firstOrderForce[i-1][SOM_DIM-1] + firstOrderForce[i][SOM_DIM-2])/3.0;
		blurredForce[0][i] = (firstOrderForce[0][i+1] + firstOrderForce[0][i-1] + firstOrderForce[1][i])/3.0;
		blurredForce[SOM_DIM-1][i] = (firstOrderForce[SOM_DIM-1][i+1] + firstOrderForce[SOM_DIM-1][i-1] + firstOrderForce[SOM_DIM-2][i])/3.0;
	}

	blurredForce[0][0] = (firstOrderForce[0][1] + firstOrderForce[1][0])/2.0;
	blurredForce[0][SOM_DIM-1] = (firstOrderForce[0][SOM_DIM-2] + firstOrderForce[1][SOM_DIM-1])/2.0;
	blurredForce[SOM_DIM-1][0] = (firstOrderForce[SOM_DIM-2][0] + firstOrderForce[SOM_DIM-1][1])/2.0;
	blurredForce[SOM_DIM-1][SOM_DIM-1] = (firstOrderForce[SOM_DIM-2][SOM_DIM-1] + firstOrderForce[SOM_DIM-1][SOM_DIM-2])/2.0;

	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			firstOrderForce[i][j] = (1.0-neighborhoodBlur)*firstOrderForce[i][j] - neighborhoodBlur*blurredForce[i][j];


	// Accumulate forces to totals. The total force is moving the particles.
	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			totalForce[i][j] = dataGain*centroidForce[i][j] + neighborhoodGain*firstOrderForce[i][j];


	// Update neuron states with a simple epsilon * target algorithm.
	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			som[i][j].z += timestep * totalForce[i][j];
}

// Evaluates the SOM at position x,y and returns the interpolated z value.
double Som2D::evaluate(Vec2f v)
{
	return evaluate(v.x, v.y);
}

// Evaluates the SOM at position x,y and returns the interpolated z value.
double Som2D::evaluate(double x, double y)
{
	// If these bounds are in effect, the network will not extrapolate values
	// outside the grid, but return the value at the grid border.
	//x = qBound(xmin, x, xmax);
	//y = qBound(ymin, y, ymax);

	// Calculate the base neighbor index.
	int i = qBound(0, int((SOM_DIM-1) * (x - xmin)/(xmax - xmin)), SOM_DIM-2);
	int j = qBound(0, int((SOM_DIM-1) * (y - ymin)/(ymax - ymin)), SOM_DIM-2);

	// Evaluate the SOM using linear interpolation between the neighbor neurons.
	double factorX = (x - som[i][j].x) / (som[i+1][j].x - som[i][j].x);
	double factorY = (y - som[i][j].y) / (som[i][j+1].y - som[i][j].y);
	double z1 = som[i][j].z + factorX*(som[i+1][j].z - som[i][j].z);
	double z2 = som[i][j+1].z + factorX*(som[i+1][j+1].z - som[i][j+1].z);
	double z = z1 + factorY*(z2-z1);

	return z;
}

void Som2D::draw(double r, double g, double b)
{
	Vec3f color;
	Vec3f color1(0.3, 0.3, 0.3);
	Vec3f color2(r, g, b);
	double transparency = 0.3;


	// Draw the training data.
	glPointSize(5.0);
	glBegin( GL_POINTS );
	glColor3f(color2.x, color2.y, color2.z);
	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			for (int k = 0; k < bin[i][j].size(); k++)
				glVertex3f(bin[i][j][k].x, bin[i][j][k].y, bin[i][j][k].z);
	glEnd();

	// Draw the nodes.
	glPointSize(5.0);
	glBegin( GL_POINTS );
	for (int i = 0; i < SOM_DIM; i++)
	{
		for (int j = 0; j < SOM_DIM; j++)
		{
			color = (stddev[i][j] == 0) ? color2 : color1 + (color2 - color1) * stddev[i][j]/maxStdDev;
			glColor3f(color.x, color.y, color.z);
			glVertex3f(som[i][j].x, som[i][j].y, som[i][j].z);
		}
	}
	glEnd();

	// Lines connecting the nodes.
	glLineWidth(0.8);
	glBegin( GL_LINES );
	for (int i = 0; i < SOM_DIM-1; i++)
	{
		for (int j = 0; j < SOM_DIM-1; j++)
		{
			color = (stddev[i][j] == 0) ? color2 : color1 + (color2 - color1) * stddev[i][j]/maxStdDev;
			glColor3f(color.x, color.y, color.z);
			glVertex3f(som[i][j].x, som[i][j].y, som[i][j].z);

			color = (stddev[i+1][j] == 0) ? color2 : color1 + (color2 - color1) * stddev[i+1][j]/maxStdDev;
			glColor3f(color.x, color.y, color.z);
			glVertex3f(som[i+1][j].x, som[i+1][j].y, som[i+1][j].z);

			color = (stddev[i][j] == 0) ? color2 : color1 + (color2 - color1) * stddev[i][j]/maxStdDev;
			glColor3f(color.x, color.y, color.z);
			glVertex3f(som[i][j].x, som[i][j].y, som[i][j].z);

			color = (stddev[i][j+1] == 0) ? color2 : color1 + (color2 - color1) * stddev[i][j+1]/maxStdDev;
			glColor3f(color.x, color.y, color.z);
			glVertex3f(som[i][j+1].x, som[i][j+1].y, som[i][j+1].z);
		}
	}
	glEnd();


	// Quads to fill the square space between the nodes.
	glBegin( GL_QUADS );
	for (int i = 0; i < SOM_DIM-1; i++)
	{
		for (int j = 0; j < SOM_DIM-1; j++)
		{
			color = (stddev[i][j] == 0) ? color2 : color1 + (color2 - color1) * stddev[i][j]/maxStdDev;
			glColor4f(color.x, color.y, color.z, transparency); // color interpolation requires glShadeModel(GL_SMOOTH)
			glVertex3f(som[i][j].x, som[i][j].y, som[i][j].z);

			color = (stddev[i+1][j] == 0) ? color2 : color1 + (color2 - color1) * stddev[i+1][j]/maxStdDev;
			glColor4f(color.x, color.y, color.z, transparency);
			glVertex3f(som[i+1][j].x, som[i+1][j].y, som[i+1][j].z);

			color = (stddev[i+1][j+1] == 0) ? color2 : color1 + (color2 - color1) * stddev[i+1][j+1]/maxStdDev;
			glColor4f(color.x, color.y, color.z, transparency);
			glVertex3f(som[i+1][j+1].x, som[i+1][j+1].y, som[i+1][j+1].z);

			color = (stddev[i][j+1] == 0) ? color2 : color1 + (color2 - color1) * stddev[i][j+1]/maxStdDev;
			glColor4f(color.x, color.y, color.z, transparency);
			glVertex3f(som[i][j+1].x, som[i][j+1].y, som[i][j+1].z);
		}
	}
	glEnd();
}

void Som2D::debug()
{
	for (int i = 0; i < SOM_DIM; i++)
		for (int j = 0; j < SOM_DIM; j++)
			qDebug() << i << j << som[i][j].x << som[i][j].y << som[i][j].z;
}
