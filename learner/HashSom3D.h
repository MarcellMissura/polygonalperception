#ifndef HASHSOM3D_H_
#define HASHSOM3D_H_
#include <QMutex>
#include "util/NVec.h"
#include "util/ColorUtil.h"

class HashSom3D
{
	static const int DIM = 3;

    int N; // Number of nodes per dimension.
	double dataGain; // How strongly the som neurons are pulled towards the data centroids.
	double neighborhoodGain; // How strongly the som neurons are pulled towards their neighborhood centroids.
	double timestep; // Simulation time step.
	int maxBinSize; // The maximum number of points allowed to be in a bin. Older points are discarded.
	int neighborHoodRadius;

	ColorUtil colorUtil;

public:

	NVec<DIM> min; // Data range min values.
	NVec<DIM> max; // Data range max values.

public:

	QVector< NVec<DIM> > raster; // Som raster coordinates on the diagonal.
	QHash<quint64, QList< NVec<DIM> > > binX; // Data bins only used for drawing.
	QHash<quint64, QList<double> > binY; // Data bins only used for drawing.
	QHash<quint64, double> centroid; // Centroids of the data bins. Only where there is data.
	QHash<quint64, double> centroidForce; // The currently acting centroid force. Only there there is data.

	QHash<quint64, double> som; // The output values of the som neurons.
	QHash<quint64, bool> active;
	QHash<quint64, double> neighborHoodForce; // The currently acting first order force on each neuron.

	quint64 lastNNhk; // Multiple bin hit filter.

	QMutex drawMutex; // Only for drawing.
	QMutex mutex; // Data point queue.
	QList< NVec<DIM> > Xq;
	QList<double> Yq;

	bool showDataPoints;
	bool showActivity;
	bool nothingToDo;

public:

	HashSom3D();
    ~HashSom3D(){}

	void setMinMax(double minx1, double maxx1, double minx2, double maxx2, double minx3, double maxx3);
	void setMinMax(NVec<DIM> minn, NVec<DIM> maxx);
	void setBins(int bins);
	void setDataGain(double dataGain);
	void setNeighborhoodGain(double neighborhoodGain);
	void setTimestep(double timestep);
	void setMaxBinSize(int maxBinSize);
	void setNeighborHoodRadius(int radius);

	void init();
    void reset(){init();}

	void addPoint(NVec<DIM> x, double y);
	void addPoints(QList< NVec<DIM> > _X, QList<double> _y);
	double evaluateAt(NVec<DIM> x);
	int train(int maxIterations=1);
	void draw(double x3 = 0);

	double metric();
	void query();

public:
	void save(QString name = "HASHSOM3D");
	void load(QString name = "HASHSOM3D");
	void printMap(QString name = "data/sommap.txt", double x3 = 0);

private:
	void trainingStep();
	quint64 hashKey(int i, int j, int k);
	quint64 hashKey(NVec<DIM> x);
	QList<quint64> enumerateNeighborHood(NVec<DIM> X, int radius = 1);

};


#endif /* HASHSOM3D_H_ */
