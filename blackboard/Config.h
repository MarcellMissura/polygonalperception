#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <QList>
#include <QHash>
#include <QString>
#include <stdint.h>

struct Config
{
    double rcIterationTime;
    double debugLevel;
    double bufferSize;

    double floor;
    double ceiling;

    double gridSize;
    double gridX;
    double gridY;
    double minimumSegmentSize;
    double douglasPeuckerEpsilon;
    double dilationRadius;
    double levelCount;

    double samplesX;
    double samplesY;
    double pruneThreshold;
    double floodThreshold;
    double mergeThreshold;

    double floorDz;
    double heightmapDz;
    double polygonsDz;
	
	Config();
    ~Config(){}

	void init();
    void save(QString name = "");
    void load(QString name = "");

	double& operator[](int i);
	double& operator()(int i);
	double& operator[](QString key);
	double& operator()(QString key);

private:

	double sink;

    // Registers a member variable for index based access.
    void registerMember(QString name, double* member, double sliderFactor)
    {
        memberNames << name;
        memberOffsets[name] = (quint64)member - (quint64)this;
        sliderFactors[name] = sliderFactor/100;
    }

    QHash<QString, ptrdiff_t> memberOffsets;

public:
    QList<QString> memberNames; // Contains the names of the members in the right order.
	QHash<QString, double> sliderFactors; // The factors of all explicitely registered config variables.
};

extern Config config;

#endif /* CONFIGURATION_H_ */
