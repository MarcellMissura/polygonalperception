#include "Config.h"
#include "globals.h"
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>

// The global config object contains application wide configuration variables.
// Basically, the config object is a globally accessible struct with public
// read and write access for everyone (not thread safe!). Yes, even if this
// violates common programming paradigms, it focuses on simplicity of use.
// Just include the config header anywhere and use config.blabla to access a
// configuration parameter. Typically you will not want to write any parameters
// during runtime, only the config slider widget wants to do so.
// Similar to the State object, the Config object provides some basic reflection
// capabilities so that config sliders can be automatically generated for the gui.
// This is also used to automatically generate a config file that can be saved and
// loaded to preserve the config variables.
// All config variables are declared in this central place. Declare them in the
// config.h header, initialize them in the constructor and optionally register
// them in the init() method if you want a slider to be created and the config
// variable to be saved in the file. Every registered config variable gets a name
// and a slider factor assigned that determines the sensitivity of the slider.
// The config object also supports save() and load() functions that serializes
// and unserializes the variable contents in a hand editable text file. The save
// and load functions take a robot name as an argument to support different config
// sets for different robots.

Config config;

Config::Config()
{
    rcIterationTime = 0.05;
    debugLevel = -1;
    bufferSize = 10;

    gridSize = 100;
    gridX = 5.0;
    gridY = 2.5;
    douglasPeuckerEpsilon = 0.7;
    dilationRadius = 0.3;
    floor = 0.05;
    ceiling = 0.5;
    minimumSegmentSize = 1;
    levelCount = 4;

    samplesX = 32;
    samplesY = 32;
    pruneThreshold = 0.8;
    floodThreshold = 0.01;
    mergeThreshold = 0.1;

    floorDz = 0;
    heightmapDz = 0;
    polygonsDz = 0;
}

// The init() method should be called after construction.
// Here, all config variables are registered to build a descriptor meta
// structure that allows index and key based access to their values.
// If you don't want to see a certain member on the gui, there is no
// need to register it.
void Config::init()
{
    registerMember("systemIterationTime", &rcIterationTime, 1.0);
    registerMember("debugLevel", &debugLevel, 100.0);
    registerMember("bufferSize", &bufferSize, 4000.0);

    registerMember("heightmap.gridSize", &gridSize, 1000);
    registerMember("heightmap.gridX", &gridX, 10);
    registerMember("heightmap.gridY", &gridY, 10);
    registerMember("heightmap.epsilonDouglasPeucker", &douglasPeuckerEpsilon, 2.0);
    registerMember("heightmap.dilationRadius", &dilationRadius, 1.0);
    registerMember("heightmap.floor", &floor, 0.1);
    registerMember("heightmap.ceiling", &ceiling, 2.00);
    registerMember("heightmap.minimumSegmentSize", &minimumSegmentSize, 10.00);
    registerMember("heightmap.levelCount", &levelCount, 100.0);

    registerMember("floordetection.samplesX", &samplesX, 100.0);
    registerMember("floordetection.samplesY", &samplesY, 100.0);
    registerMember("floordetection.pruneThreshold", &pruneThreshold, 1.0);
    registerMember("floordetection.greedThreshold", &floodThreshold, 0.1);
    registerMember("floordetection.mergeThreshold", &mergeThreshold, 0.2);

    registerMember("gui.floor", &floorDz, 0.2);
    registerMember("gui.heightmap_dz", &heightmapDz, 0.2);
    registerMember("gui.polygons_dz", &polygonsDz, 0.2);
}

// Loads the config variables from the .conf file.
// Unregistered variables are ignored.
void Config::load(QString name)
{
    if (name.isEmpty())
        name = "config";

    QFile file("conf/" + name + ".conf");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Couldn't load config file" << file.fileName();
		return;
	}

	QTextStream in(&file);
	QString line;
	QStringList list;
	bool ok;
	while (!in.atEnd())
	{
		line = in.readLine().trimmed();
		list = line.split("=");
		if (list.length() == 2 && !line.startsWith("//") && !line.startsWith("#"))
		{
			QString key = list[0].trimmed();
			double value = list[1].trimmed().toDouble(&ok);
			if (memberNames.contains(key))
				this->operator[](key) = value;
		}
	}

	file.close();
}

// Saves the config variables to the .conf file.
void Config::save(QString name)
{
    if (name.isEmpty())
        name = "config";

    QFile file("conf/" + name + ".conf");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		qDebug() << "Couldn't open config file" << file.fileName();
		return;
	}

	QTextStream out(&file);
	foreach (QString key, memberNames)
		out << key << "=" << QString::number(this->operator[](key)) << endl;
	file.close();
}

// Returns a reference to the ith member of this object.
double& Config::operator()(int i)
{
	return this->operator[](memberNames[i]);
}

// Returns a reference to the ith member of this object.
double& Config::operator[](int i)
{
	return this->operator[](memberNames[i]);
}

// Returns a reference to the member that was registered with the given key.
double& Config::operator()(QString key)
{
	return this->operator[](key);
}

// Returns a reference to the member that was registered with the given key.
// If you try to access an unregistered member, you will get a useless reference to a black hole and a warning.
double& Config::operator[](QString key)
{
	if (!memberNames.contains(key))
	{
		qDebug() << "You are trying to access a non existent config member" << key;
		return sink;
	}

	double* ptr = (double*)((uintptr_t)this+memberOffsets[key]);
	double& rf = *ptr;
	return rf;
}
