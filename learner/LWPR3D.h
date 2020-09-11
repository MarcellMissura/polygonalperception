#ifndef LWPR3D_H_
#define LWPR3D_H_
#include <QMutex>
#include "util/NVec.h"
#include "util/ColorUtil.h"
#include <lwpr.hh>

class LWPR3D
{
	static const int DIM = 3;
    int N;
	int maxBinSize; // The maximum number of points allowed to be in a bin. Older points are discarded.

    double kernelSize;
    bool kernelSizeAdaptation;
    double learningRate;

	ColorUtil colorUtil;
	LWPR_Object* model;
	QMutex mutex;

public:

	QString name;

	NVec<DIM> min; // Data range min values.
	NVec<DIM> max; // Data range max values.

    QVector< NVec<DIM> > raster; // Raster coordinates on the diagonal.
    QVector<double> Y; // Ouput values of all grid nodes in a flat list (size: pow(N, DIM)-ish).
    QVector<double> C; // Confidence values of all grid nodes in a flat list (size: pow(N, DIM)-ish).
	QHash<quint64, QList< NVec<DIM> > > binX; // Data bins only needed for drawing.
	QHash<quint64, QList<double> > binY; // Data bins only needed for drawing.

public:

	LWPR3D();
    ~LWPR3D(){delete model;}

    void init();
    void reset(){init();}

    void setKernelSize(double ks);
    void setKernelSizeAdaptation(bool ksa);
    void setLearningRate(double lr);

	void setMinMax(double minx1, double maxx1, double minx2, double maxx2, double minx3, double maxx3);
	void setMinMax(NVec<DIM> minn, NVec<DIM> maxx);
    void setN(int N);
	void setMaxBinSize(int maxBinSize);
    void rasterize();

    void loadTxt(char *fileName = "data/LWPR3D.txt");
    void save(char *fileName = "data/LWPR3D.bin");
    void load(char *fileName = "data/LWPR3D.bin");

    void addDataPoint(NVec<DIM> x, double y);
    void addData(QList< NVec<DIM> > X, QList<double> Y);
    void prepareDrawing();

	double evaluateAt(NVec<DIM> x);
	double evaluateAt(NVec<DIM> x, double& confidence);

    void draw(double z=0, int sampleFactor=1, double zScale=1.0, double colorContrast=1.0, double transparency=0, bool showData=false);
	double metric();
    void printReceptiveFields();
    void logMap(QString name = "data/lwprmap.txt", double x3=0);
    void logData(QString name = "data/lwprmap_data.txt", double x3=0);

private:
	quint64 hashKey(int i, int j, int k);
	quint64 hashKey(NVec<DIM> x);

    void drawReceptiveFields();

};


#endif /* LWPR3D_H_ */
