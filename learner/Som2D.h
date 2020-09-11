#ifndef SOM2D_H_
#define SOM2D_H_
#include <QString>
#include <util/Vec3f.h>
#include <util/Vec2f.h>

class Som2D
{
	static const int SOM_DIM = 50;

public:
	double dataGain; // How strongly the som neurons are pulled towards the data centroids.
	double neighborhoodGain; // How strongly the som neurons are pulled towards their neighborhood centroids.
	double neighborhoodBlur; // Between 0 and 1. How strong is the blur filter that is applied to the first order forces.
	double timestep; // Simulation time step.
	int maxBinSize; // The maximum number of points allowed to be in a bin. Older points are discarded.
	int bins; // Number of nodes (bins) per dimension.

	double xmax; // Data range max value.
	double xmin; // Data range min value.
	double ymax; // Data range max value.
	double ymin; // Data range min value.


private:

	QList<Vec3f> bin[SOM_DIM][SOM_DIM];// Data bins assigned to the som neurons.
	Vec3f som[SOM_DIM][SOM_DIM]; // The actual som neurons, x,y are the input values, z is the output.
	double centroid[SOM_DIM][SOM_DIM]; // Centroids of the data bins assigned to the som neurons.
	double stddev[SOM_DIM][SOM_DIM]; // The standard deviation of the centroids in each of the data bins.
	double centroidForce[SOM_DIM][SOM_DIM]; // The currently acting centroid force on each neuron.
	double firstOrderForce[SOM_DIM][SOM_DIM]; // The currently acting first order force on each neuron.
	double blurredForce[SOM_DIM][SOM_DIM];
	double totalForce[SOM_DIM][SOM_DIM]; // The net currently acting force on each neuron.
	double maxStdDev;

public:

	Som2D();
	~Som2D(){};

	void setMinMax(double minx, double maxx, double miny, double maxy);
	void setBins(int bins);
	void setDataGain(double dataGain);
	void setNeighborhoodGain(double neighborhoodGain);
	void setNeighborhoodBlur(double neighborhoodBlur);
	void setTimestep(double timestep);
	void setMaxBinSize(int maxBinSize);

	void init();
	void reset(){init();};
	void clear();

	void addPoint(Vec3f v);
	void addPoint(double x, double y, double z);
	void train(int howManySteps=1);
	double evaluate(Vec2f v);
	double evaluate(double x, double y);
	void draw(double r = 1.0, double g = 1.0, double b = 1.0);
	void debug();

public:
	void save(QString name = "SOM2D");
	void load(QString name = "SOM2D");

private:
	void trainingStep();

};


#endif /* SOM2D_H_ */
