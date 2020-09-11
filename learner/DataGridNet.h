#ifndef DATAGRIDNET_H_
#define DATAGRIDNET_H_
#include "GridNet.h"
#include <util/VecN.h>
#include <util/ColorUtil.h>
#include "GL/gl.h"
#include "util/GLlib.h"
#include <QFile>
#include <QMutex>

// The DataGridNet extends the GridNet with data storage capabilities.
// Data points can be added to the grid. They will be sorted into bins
// assigned to the grid nodes. Each bin holds one data point. When a
// a new point is added, it replaces the old data point according to a
// set behavior:
// - always replace (keep newest)
// - replace if smaller (keep minimum)
// - replace if greater (keep maximum)
// - replace with running average (keep mean)

template <uint DIM=3>
class DataGridNet : public GridNet<DIM>
{
    int behavior;

protected:

    QHash<uint, VecN<DIM+1> > bin;// Data bins assigned to the grid nodes.
    QHash<uint, uint> counts; // Mins of the data bins.

    QHash<uint, bool> active; // Activatable?

public:

    enum Behavior { replaceAlways, replaceMinimum, replaceMaximum, replaceMean };

    DataGridNet();
    ~DataGridNet(){}

    virtual void reset(); // Resets to a blank state.

    virtual void load(QString name = "DataGridNet.dat");
    virtual void save(QString name = "DataGridNet.dat");

    // Binned data storage interface.
    void setBehavior(int b);
    virtual uint addDataPoint(const VecN<DIM> &x, const double y);
    void addDataPoints(const QList< VecN<DIM> > &X, const QList<double> &y);
    void addDataPoints(const QList< VecN<DIM+1> > &X);
    void clearDataPoints();
    void eraseDataPoints(const VecNu<DIM> &idx, uint r=0);
    void eraseDataPoints(uint idx, uint r=0);
    virtual QList< VecN<DIM+1> > getDataPoints(const VecNu<DIM> &idx, uint r=0);
    virtual QList< VecN<DIM+1> > getDataPoints(uint idx, uint r=0);
    virtual QList< VecN<DIM+1> > getAllDataPoints();

    // Activation.
    void activate(const VecNu<DIM> &idx, uint r=0);
    void activate(uint idx, uint r=0);

    // Basic data to node transfer.
    void setDataToNodeOutputs();

    // OpenGL drawing code.
    void drawGridSlice(double z=0, uint sampleFactor=1, double zScale=1.0, double colorContrast=1.0, double transparency=0, bool showData=false);
    void drawData(double colorContrast, uint sampleFactor=1);
    void drawActive();
};

template <uint DIM>
DataGridNet<DIM>::DataGridNet()
{
    behavior = Behavior::replaceAlways;
}

// Resets the data grid to a blank state: zero output, zero confidence.
template <uint DIM>
void DataGridNet<DIM>::reset()
{
    GridNet<DIM>::reset();
    clearDataPoints();
    active.clear();
}

// Resets the data grid to a blank state: zero output, zero confidence.
template <uint DIM>
void DataGridNet<DIM>::setBehavior(int b)
{
    behavior = b;
}

// Loads a binary saved DataGridNet.
template <uint DIM>
void DataGridNet<DIM>::load(QString name)
{
    GridNet<DIM>::load(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "DataGridNet::load(): invalid file name" << name;
        return;
    }
    fileName += ".dgnt";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "DataGridNet::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> bin;
    in >> counts;
    file.close();
}

// Saves the DataGridNet in a binary file.
template <uint DIM>
void DataGridNet<DIM>::save(QString name)
{
    GridNet<DIM>::save(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "DataGridNet::save(): invalid file name" << name;
        return;
    }
    fileName += ".dgnt";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "DataGridNet::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << bin;
    out << counts;
    file.close();
}

// Batch adds a bunch of data points.
template <uint DIM>
void DataGridNet<DIM>::addDataPoints(const QList<VecN<DIM> > &X, const QList<double> &y)
{
    for (int i = 0; i < X.size(); i++)
        addDataPoint(X[i], y[i]); // This should not work well when overriding.
}

// Batch adds a bunch of data points.
template <uint DIM>
void DataGridNet<DIM>::addDataPoints(const QList<VecN<_Tp1> > &X)
{
    for (int i = 0; i < X.size(); i++)
        addDataPoint(X[i], X[i][DIM]); // This should not work well when overriding.
}

// Adds a single data point with input values x and output value y to the data set.
// The method returns the flat index of the grid node the data point was assigned to.
// Out of bounds points are ignored and -1 is returned.
template <uint DIM>
uint DataGridNet<DIM>::addDataPoint(const VecN<DIM> &x, const double y)
{
    // Discard NaN cases.
    if (x != x || y != y)
        return -1;

    // Discard out of range points.
    for (uint d = 0; d < DIM; d++)
        if (x[d] > this->max[d] || x[d] < this->min[d])
            return -1;

    // Calculate the index of the grid node closest to x.
    uint n = Grid<DIM>::getNodeFlatIndex(x); // make sure to use an untransformed version here.

    // Sort the new point into the bin.
    if (behavior == Behavior::replaceAlways)
    {
        bin[n] = VecN<DIM+1>(x, y);
    }
    if (behavior == Behavior::replaceMinimum)
    {
        if (counts[n] == 0 || y < bin[n].w)
            bin[n] = VecN<DIM+1>(x, y);
    }
    if (behavior == Behavior::replaceMaximum)
    {
        if (counts[n] == 0 || y > bin[n].w)
            bin[n] = VecN<DIM+1>(x, y);
    }
    if (behavior == Behavior::replaceMean)
    {
        bin[n] = VecN<DIM+1>(x, (bin[n].w*counts[n]+y)/(counts[n]+1));
    }

    counts[n]++;

    return n;
}

// Clears the data bins, but does not change the output values.
template <uint DIM>
void DataGridNet<DIM>::clearDataPoints()
{
    bin.clear();
    counts.clear();
}

// Erases all data in a neighbourhood of radius r around the grid cell identified by the DIM index idx.
template <uint DIM>
void DataGridNet<DIM>::eraseDataPoints(const VecNu<DIM> &idx, uint r)
{
    uint n = this->convertIndex(idx);
    eraseDataPoints(n, r);
}

// Erases all data in a neighbourhood of radius r around the grid cell identified by the flat index n.
template <uint DIM>
void DataGridNet<DIM>::eraseDataPoints(uint n, uint r)
{
    QVector<uint> nn = this->enumerateNeighborHood(n, r);
    for (int i=0; i < nn.size(); i++)
    {
        bin.remove(nn[i]);
        counts.remove(nn[i]);
    }
}

// Returns the data points in a neighbourhood of radius r around the grid node identified by the DIM index idx.
template <uint DIM>
QList< VecN<DIM+1> > DataGridNet<DIM>::getDataPoints(const VecNu<DIM> &idx, uint r)
{
    QList< VecN<DIM+1> > data;
    QVector<uint> nn = this->enumerateNeighborHood(idx, r);
    for (int i=0; i < nn.size(); i++)
        if (bin.contains(nn[i]))
            data << bin[nn[i]];
    return data;
}

// Returns the data points in a neighbourhood of radius r around the grid node identified by the flat index n.
template <uint DIM>
QList< VecN<DIM+1> > DataGridNet<DIM>::getDataPoints(uint n, uint r)
{
    QList< VecN<DIM+1> > data;
    QVector<uint> nn = this->enumerateNeighborHood(n, r);
    for (int i=0; i < nn.size(); i++)
        if (bin.contains(nn[i]))
            data << bin[nn[i]];
    return data;
}

// Returns all data points stored in the grid.
template <uint DIM>
QList< VecN<DIM+1> > DataGridNet<DIM>::getAllDataPoints()
{
    return bin.values();
}


// Sets the active flag to true for all cells in a neighbourhood of radius r around the grid cell identified by the DIM index idx.
template <uint DIM>
void DataGridNet<DIM>::activate(const VecNu<DIM> &idx, uint r)
{
    uint n = this->convertIndex(idx);
    activate(n, r);
}

// Sets the active flag to true for all cells in a neighbourhood of radius r around the grid cell identified by the flat index n.
template <uint DIM>
void DataGridNet<DIM>::activate(uint n, uint r)
{
    QVector<uint> nn = this->enumerateNeighborHood(n, r);
    for (int i=0; i < nn.size(); i++)
        active[nn[i]] = true;
}


// Set the grid node outputs to the means of the data bins whereever applicable.
template <uint DIM>
void DataGridNet<DIM>::setDataToNodeOutputs()
{
    QList<uint> keys = bin.keys();
    for (int i = 0; i < keys.size(); i++)
        this->setNodeOutput(keys[i], bin[keys[i]].w, 1);
}

// OpenGL drawing code that draws the location of the stored data points.
template <uint DIM>
void DataGridNet<DIM>::drawData(double colorContrast, uint sampleFactor)
{
    sampleFactor = qMax((uint)1, sampleFactor);

    QColor color;
    glPointSize(4);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    QList<uint> keys = bin.keys();
    for (int i = 0; i < keys.size(); i=i+sampleFactor)
    {
        color = colorUtil.getHeightMapColor(bin[keys[i]].w, 0, colorContrast);
        glColor3f(color.redF(), color.greenF(), color.blueF());
        glVertex3f(bin[keys[i]].x, bin[keys[i]].y, bin[keys[i]].z);
    }
    glEnd();
}

// OpenGL drawing code that draws the location of the grid nodes.
template <uint DIM>
void DataGridNet<DIM>::drawActive()
{
    // Draw the active neurons.
    glPointSize(5);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    QList<uint> keys = active.keys();
    for (int i = 0; i < keys.size(); i++)
    {
        VecN<DIM> v = Grid<DIM>::getNodeCoordinates(keys[i]);
        glVertex3f(v.x, v.y, v.z);
    }
    glEnd();
}


// OpenGL drawing code that draws a 2D slice of the DataGridNet.
// No interpolation is used, just the output values at the DataGridNet coordinates.
// The z input parameter should be in [0,1]. It will be mapped to the "vertical"
// index of the DataGridNet slice.
template <uint DIM>
void DataGridNet<DIM>::drawGridSlice(double z, uint sampleFactor, double zScale, double colorContrast, double transparency, bool showData)
{
    GridNet<DIM>::drawGridSlice(z, sampleFactor, zScale, colorContrast, transparency);

    // Draw the data points.
    if (showData)
    {
        // Compute "vertical" slice index.
        uint k = qBound((uint)0, uint(z*this->N[2]-1), this->N[2]-1);
        sampleFactor = qMax(sampleFactor, (uint)1);

        glPushMatrix();
        VecNu<DIM> idx(0, 0, k);
        glTranslated(0, 0, Grid<DIM>::getNodeCoordinates(idx).z); // use the untransformed version here

        glPushMatrix();
        glScaled(1.0, 1.0, zScale);

        glPointSize(3);
        glColor3f(0,0,0);
        glBegin(GL_POINTS);
        for (uint j = 0; j < this->N[1]; j++)
        {
            for (uint i = 0; i < this->N[0]; i++)
            {
                VecNu<DIM> idx(i, j, k);
                uint n = this->convertIndex(idx);
                if (bin.contains(n))
                    glVertex3f(bin[n].x, bin[n].y, bin[n].w);
            }
        }
        glEnd();

        glPopMatrix();

        glPopMatrix();
    }
}

#endif /* DATAGRIDNET_H_ */
