#ifndef SOM1D_H_
#define SOM1D_H_
#include <QString>
#include <util/Vec2f.h>


class Som1D
{
	static const int SOM_DIM = 50;

public:
	double dataGain; // How strongly the som neurons are pulled towards the data centroids.
	double neighborhoodGain; // How strongly the som neurons are pulled towards their neighborhood centroids.
	double neighborhoodBlur; // Between 0 and 1. How strong is the blur filter that is applied to the first order forces.
	double timestep; // Simulation time step.
	int maxBinSize; // The maximum number of points allowed to be in a bin. Older points are discarded.

	double xmax; // Data range max value.
	double xmin; // Data range min value.

public:

	QList<Vec2f> bin[SOM_DIM];// Data bins assigned to the som neurons.
	Vec2f som[SOM_DIM]; // The actual som neurons, x,y are the input values, z is the output.
	double centroid[SOM_DIM]; // Centroids of the data bins assigned to the som neurons.
	double stddev[SOM_DIM]; // The standard deviation of the centroids in each of the data bins.
	double centroidForce[SOM_DIM]; // The currently acting centroid force on each neuron.
	double firstOrderForce[SOM_DIM]; // The currently acting first order force on each neuron.
	double blurredForce[SOM_DIM];
	double totalForce[SOM_DIM]; // The net currently acting force on each neuron.
	double maxStdDev;

public:

	Som1D();
	~Som1D(){};

	void init();
	void reset(){init();};
	void clear();
	void setMinMax(double minn, double maxx);
	void save(QString name = "SOM1D");
	void load(QString name = "SOM1D");

	void addPoint(Vec2f v);
	void addPoint(double x, double y);
	void train(int howManySteps=1);
	double evaluate(double xx);
	void draw(double r = 1.0, double g = 1.0, double b = 1.0);

private:
	void trainingStep();

};


#endif /* SOM1D_H_ */
