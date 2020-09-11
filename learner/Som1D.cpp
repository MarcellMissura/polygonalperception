#include "Som1D.h"
#include <math.h>
#include <QGLViewer/qglviewer.h>
#include <QFile>
#include <QDebug>

// The SOM neural network is a simple learner for 2D input dimensions and 1 output dimension.
// The network is a lattice evenly spread out in the data range. The nodes (neurons) of the lattice
// are pulled towards the centroid of the data in the vicinity, but also towards the centroid of
// their neighborhood neurons.

Som1D::Som1D()
{
	dataGain = 1.0; // How strongly the som neurons are pulled towards the data centroids.
	neighborhoodGain = 3.0; // How strongly the som neurons are pulled towards their neighborhood centroids.
	neighborhoodBlur = 0.5; // Between 0 and 1. How strong is the blur filter that is applied to the neighborhood forces.
	timestep = 0.2; // Simulation time step. The larger the faster, but the more likely to explode.
	maxBinSize = 100;  // The maximum number of points allowed to be in a bin. Older points are discarded.

	maxStdDev = 0;

	xmax = 0; // Data range limit.
	xmin = 0; // Data range limit.

	// Initialize the data structures with a SOM_DIM x SOM_DIM lattice filled with all zeros.
	memset(som, 0, SOM_DIM*sizeof(Vec2f));
	memset(centroid, 0, SOM_DIM*sizeof(double));
	memset(stddev, 0, SOM_DIM*sizeof(double));
	memset(centroidForce, 0, SOM_DIM*sizeof(double));
	memset(firstOrderForce, 0, SOM_DIM*sizeof(double));
	memset(blurredForce, 0, SOM_DIM*sizeof(double));
	memset(totalForce, 0, SOM_DIM*sizeof(double));
}

// Sets the min and max boundaries of the data range. Do this first.
void Som1D::setMinMax(double minn, double maxx)
{
	xmin = minn;
	xmax = maxx;
}

// Initializes (or resets) the som grid with a SOM_DIM lattice evenly distributed in
// the data range. Min and max boundaries have to be provided beforehand with the setMinMax() function.
// The z values of the som neurons are initialized with 0. This does not clear the data bins.
void Som1D::init()
{
	// Move the som neurons in place and initialize them with 0.
	for (int i = 0; i < SOM_DIM; i++)
	{
		som[i].x = xmin + (xmax-xmin)*(double)i/(SOM_DIM-1);
		som[i].y = 0.0;
	}
}

// Clears the data bins, but does not reset the SOM neurons.
void Som1D::clear()
{
	for (int i = 0; i < SOM_DIM; i++)
		bin[i].clear();
	memset(centroid, 0, SOM_DIM*sizeof(double));
	memset(stddev, 0, SOM_DIM*sizeof(double));
	maxStdDev = 0;
}

// Saves the SOM lattice, but not the data.
void Som1D::save(QString name)
{
	QFile file("data/" + name + ".som");
	file.open(QIODevice::WriteOnly);
	QDataStream out(&file);
//	out.setVersion(QDataStream::Qt_3_1);

	out << xmin;
	out << xmax;

	for (int i = 0; i < SOM_DIM; i++)
	{
		out << som[i].x;
		out << som[i].y;
	}

	file.close();
}

// Loads the SOM. After loading, the data bins are empty.
void Som1D::load(QString name)
{
	// Open the file.
	QFile file("data/" + name + ".som");
	if (!file.open(QIODevice::ReadOnly))
		qDebug() << "Could not open file" << file.fileName();
	QDataStream in(&file);
	///in.setVersion(QDataStream::Qt_3_1);

	// Load the meta parameters.
	in >> xmin;
	in >> xmax;

	// Load the som neurons from the file.
	for (int i = 0; i < SOM_DIM; i++)
	{
		in >> som[i].x;
		in >> som[i].y;
	}

	file.close();

	// Reset the data bins.
	clear();
}

// Adds a single point to the data set. The data range boundaries must be already known.
void Som1D::addPoint(Vec2f v)
{
	// Discard nan cases.
	if (v != v)
		return;

	// Calculate the nearest neighbor bin index.
	int i = qRound((SOM_DIM-1) * (v.x - xmin)/(xmax - xmin));

	// Discard out of range points.
	if (i < 0 or i > SOM_DIM-1)
		return;

	// Append to the data bin.
	if (maxBinSize > 0)
		while (bin[i].size() > maxBinSize)
			bin[i].pop_front();
	bin[i].push_back(v);

	// Calculate the new centroid.
	centroid[i] = 0;
	for (int k = 0; k < bin[i].size(); k++)
		centroid[i] += bin[i][k].y;
	centroid[i] /= (double)bin[i].size();

	// Calculate the standard deviation as well.
	stddev[i] = 0;
	for (int k = 0; k < bin[i].size(); k++)
		stddev[i] += (bin[i][k].y - centroid[i])*(bin[i][k].y - centroid[i]);
	stddev[i] = sqrt(stddev[i]/(double)bin[i].size());
	maxStdDev = qMax(maxStdDev, stddev[i]);
}

void Som1D::addPoint(double x, double y)
{
	addPoint(Vec2f(x,y));
}

// Performs a bunch of training steps.
void Som1D::train(int howManySteps)
{
	for (int i = 0; i < howManySteps; i++)
		trainingStep();
}

// Calculates the forces acting on the neurons and moves the neurons by a bit.
void Som1D::trainingStep()
{
	// Calculate centroid forces.
	for (int i = 0; i < SOM_DIM; i++)
		centroidForce[i] = bin[i].size() > 0 ? centroid[i] - som[i].y : 0;

	// Calculate first order neighborhood forces with explicit border case handling.
	for (int i = 1; i < SOM_DIM-1; i++)
		firstOrderForce[i] = (som[i+1].y + som[i-1].y)/2.0 - som[i].y;

	firstOrderForce[0] = 2.0*som[1].y - som[2].y - som[0].y;
	firstOrderForce[SOM_DIM-1] = 2.0*som[SOM_DIM-2].y - som[SOM_DIM-3].y - som[SOM_DIM-1].y;


	// Apply a "blur filter" on the first order forces.
	for (int i = 1; i < SOM_DIM-1; i++)
		blurredForce[i] = 0.5*(firstOrderForce[i+1] + firstOrderForce[i-1]);

	blurredForce[0] = firstOrderForce[1];
	blurredForce[SOM_DIM-1] = firstOrderForce[SOM_DIM-2];

	for (int i = 0; i < SOM_DIM; i++)
		firstOrderForce[i] = (1.0-neighborhoodBlur)*firstOrderForce[i] - neighborhoodBlur*blurredForce[i];


	// Accumulate forces to totals. The total force is moving the particles.
	for (int i = 0; i < SOM_DIM; i++)
		totalForce[i] = dataGain*centroidForce[i] + neighborhoodGain*firstOrderForce[i];


	// Update neuron states with a simple epsilon * target algorithm.
	for (int i = 0; i < SOM_DIM; i++)
		som[i].y += timestep * totalForce[i];
}

// Evaluates the SOM at position x,y and returns the interpolated z value.
double Som1D::evaluate(double x)
{
	// If these bounds are in effect, the network will not extrapolate values
	// outside the grid, but return the value at the grid border.
	//x = qBound(xmin, x, xmax);

	// Calculate the base neighbor index.
	int i = qBound(0, int((SOM_DIM-1) * (x - xmin)/(xmax - xmin)), SOM_DIM-2);

	// Evaluate the SOM using linear interpolation between the neighbor neurons.
	double factorX = (x - som[i].x) / (som[i+1].x - som[i].x);
	double z = som[i].y + factorX*(som[i+1].y - som[i].y);
	return z;
}

void Som1D::draw(double r, double g, double b)
{
	Vec3f color(r, g, b);
	Vec3f color1(r, g, b);
	Vec3f color2(1.0, 0, 0);
	double transparency = 0.3;


	// Draw the training data.
	glPointSize(8.0);
	glBegin( GL_POINTS );
	glColor3f(color2.x, color2.y, color2.z);
	for (int i = 0; i < SOM_DIM; i++)
		for (int k = 0; k < bin[i].size(); k++)
			glVertex3f(bin[i][k].x, 0.0, bin[i][k].y);
	glEnd();

	// Draw the nodes.
	glPointSize(5.0);
	glBegin( GL_POINTS );
	for (int i = 0; i < SOM_DIM; i++)
	{
		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i]/maxStdDev;
		glColor3f(color.x, color.y, color.z);
		glVertex3f(som[i].x, 0.0, som[i].y);
	}
	glEnd();

	// Lines connecting the nodes.
	glLineWidth(0.8);
	glBegin( GL_LINES );
	for (int i = 0; i < SOM_DIM-1; i++)
	{
		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i]/maxStdDev;
		glColor3f(color.x, color.y, color.z);
		glVertex3f(som[i].x, 0.0, som[i].y);

		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i+1]/maxStdDev;
		glColor3f(color.x, color.y, color.z);
		glVertex3f(som[i+1].x, 0.0, som[i+1].y);
	}
	glEnd();


	// Quads to fill the square space between the nodes.
	glBegin( GL_QUADS );
	for (int i = 0; i < SOM_DIM-1; i++)
	{
		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i]/maxStdDev;
		glColor4f(color.x, color.y, color.z, transparency); // color interpolation requires glShadeModel(GL_SMOOTH)
		glVertex3f(som[i].x, 0.0, 0.0);

		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i]/maxStdDev;
		glColor4f(color.x, color.y, color.z, transparency);
		glVertex3f(som[i].x, 0.0, som[i].y);

		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i+1]/maxStdDev;
		glColor4f(color.x, color.y, color.z, transparency);
		glVertex3f(som[i+1].x, 0.0, som[i+1].y);

		if (maxStdDev > 0)
			color = color1 + (color2 - color1) * stddev[i+1]/maxStdDev;
		glColor4f(color.x, color.y, color.z, transparency);
		glVertex3f(som[i+1].x, 0.0, 0.0);

	}
	glEnd();
}
