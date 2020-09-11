#include "SparseGridNet.h"
#include <util/ColorUtil.h>
#include <QFile>
#include <QMutexLocker>
#include <QDebug>


// The SparseGridNet extends the Grid with output values at the grid nodes. Linear
// interpolation is used to query a value anywhere within the grid boundaries.
// As opposed to the GridNet that allocates all needed memory at grid contruction
// time, the SparseGridNet uses a QHash for storage and minimizes the memory
// footprint at the expense of slower lookups.

// The general use of the SparseGridNet would be to first define the min and max
// data range boundaries and N the number of SparseGridNet nodes per dimension.
// Use setMinMax() and setN() for this purpose. Then, populate
// the SparseGridNet from saved data using load(), or by feeding it
// with data using the setNodeOutputs() method. Please understand that
// after the SparseGridNet has been populated, it is not a good idea to change
// the SparseGridNet structure parameters.

// After the SparseGridNet has been constructed and populated, you can query it
// at an arbitrary location x using the evaluateAt(x) function.


SparseGridNet::SparseGridNet()
{

}

// Calculates the raster of the grid coordinates. The grid nodes are distributed between the
// respective min and max values of each dimension such that the first node is located at the
// min and the last node is located at the max. Dim, N, min, and max must be set before
// computing the raster. Make sure to set the grid parameters first and then call this method
// to prepare the grid before using it.
void SparseGridNet::rasterize()
{
    // SparseGridNet overrides Grid::rasterize() so that we can resize p here.
    Grid::rasterize();
    p.resize(this->getNodeCount());
}

// Resets the data grid to a blank state: zero output, zero confidence,
// but does not change the layout of the data grid.
void SparseGridNet::reset()
{
    QMutexLocker locker(&mutex);
    Y.clear();
    C.clear();
}

// Loads a binary saved SparseGridNet.
void SparseGridNet::load(QString name)
{
    Grid::load(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "SparseGridNet::load(): invalid file name" << name;
        return;
    }
    fileName += ".sgnt";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "SparseGridNet::load(): Could not open file" << file.fileName();
        return;
    }

    QMutexLocker locker(&mutex);

    QDataStream in(&file);
    in >> Y;
    in >> C;
    file.close();
}

// Saves the SparseGridNet in a binary file.
void SparseGridNet::save(QString name)
{
    Grid::save(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "SparseGridNet::save(): invalid file name" << name;
        return;
    }
    fileName += ".sgnt";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "SparseGridNet::save(): Could not write to file" << file.fileName();
        return;
    }

    QMutexLocker locker(&mutex);

    QDataStream out(&file);
    out << Y;
    out << C;
    file.close();
}

// Sets the output value of the SparseGridNet node identified by flat index n to y,
// and the confidence of the output to c in [0,1], 1 most confident.
void SparseGridNet::setNodeOutput(uint n, double y, double c)
{
    QMutexLocker locker(&mutex);
    Y[n] = y;
    C[n] = c;
}

// Sets the output value of the SparseGridNet node identified by DIM index idx to y,
// and the confidence of the output to c.
void SparseGridNet::setNodeOutput(const uint* idx, double y, double c)
{
    uint n = this->convertIndex(idx);
    QMutexLocker locker(&mutex);
    Y[n] = y;
    C[n] = c;
}

// Returns the SparseGridNet node output value of the node identified by flat index idx.
double SparseGridNet::getNodeOutput(uint idx) const
{
    return Y.value(idx);
}

// Returns the SparseGridNet node output value of the node identified by DIM index idx.
double SparseGridNet::getNodeOutput(const uint* idx) const
{
    uint n = this->convertIndex(idx);
    return Y.value(n);
}

// Returns the SparseGridNet node confidence value of the node identified by flat index idx.
double SparseGridNet::getNodeConfidence(uint n) const
{
    return C.value(n);
}

// Returns the SparseGridNet node confidence value of the node identified by DIM index idx.
double SparseGridNet::getNodeConfidence(const uint *idx) const
{
    uint n = this->convertIndex(idx);
    return C.value(n);
}

// Text console output of the full SparseGridNet. It prints all or howMany key, value pairs.
// It helps to debug autovivification bugs.
void SparseGridNet::printSparseGridNet(uint howMany) const
{
    QList<uint> keys = Y.keys();
    uint top = howMany == 0 ? keys.size() : qMax(howMany, (uint)keys.size());
    if (howMany == 0)
        howMany = keys.size();
    for (uint i = 0; i < top; i++)
        qDebug() << keys[i] << this->getNodeCoordinates(keys[i]) << "| y:" << Y[keys[i]] << "c:" << C[keys[i]];
}

// Evaluates the SparseGridNet at point x using linear interpolation between the
// grid nodes of the simplex that envelopes x. x is truncated to lie inside
// the boundaries of the SparseGridNet. The computations in this method are carried
// out according to the paper "A Geometric Approach to Maximum-Speed
// n-Dimensional Continuous Linear Interpolation in Rectangular SparseGridNets",
// Riccardo Rovatti, Michele Borgatti, and Roberto Guerrier.
double SparseGridNet::evaluateAt(const double* x) const
{
    // Calculate the "bottom left" corner of the cell that contains X.
    const uint* idx = this->getNodeIndexBl(x); // make sure to use an untransformed version

    // Calculate the normalized coordinates of X within the cell.
    double normcoord[DIM];
    for (uint d = 0; d < DIM; d++)
        normcoord[d] = (x[d]-this->raster[d][idx[d]])/(this->raster[d][idx[d]+1]-this->raster[d][idx[d]]);

    // Compute the permutation s of the dimensions that is associated with
    // the normalized coordinates sorted in non-increasing order.
    for (uint d = 0; d < DIM; d++)
        p[d] = std::pair<double, uint>(normcoord[d], d);
    std::sort(p.begin(), p.end(), [](std::pair<double, uint> a, std::pair<double, uint> b) {return a.first > b.first;});

    // Convert the "bottom left" corner to a flat index k.
    uint k = this->convertIndex(idx);

    // Interpolate the return value in the simplex from the sorted dimensions.
    double y = Y.value(k)*(1-p[0].first); // the first one is special
    for (uint d = 1; d < DIM; d++)
    {
        //index[s[d-1].second] += 1; // this is what's written in the paper
        //k = indexToFlatIndex(index);
        k += this->cumN[p[d-1].second]; // a faster version that maintains the flat index k
        y += Y.value(k)*(p[d-1].first-p[d].first); // fast access to flat Y array
    }
    //index[s[DIM-1].second] += 1;
    //k = indexToFlatIndex(index);
    k += this->cumN[p[DIM-1].second]; // maintain the flat index
    y += Y.value(k)*p[DIM-1].first; // the last one is also special

    return y;
}

// Evaluates the SparseGridNet at point x using linear interpolation between the
// grid nodes of the simplex that envelopes x. x is truncated to lie inside
// the boundaries of the SparseGridNet. The computations in this method are carried
// out according to the paper "A Geometric Approach to Maximum-Speed
// n-Dimensional Continuous Linear Interpolation in Rectangular SparseGridNets",
// Riccardo Rovatti, Michele Borgatti, and Roberto Guerrier. If no data is
// loaded, this method returns 0.
double SparseGridNet::evaluateAt(const double* x, double& c) const
{
    // Calculate the "bottom left" corner of the cell that contains X.
    const uint* idx = this->getNodeIndexBl(x); // make sure to use an untransformed version

    // Calculate the normalized coordinates of X within the cell.
    double normcoord[DIM];
    for (uint d = 0; d < DIM; d++)
        normcoord[d] = (x[d]-this->raster[d][idx[d]])/(this->raster[d][idx[d]+1]-this->raster[d][idx[d]]);

    // Compute the permutation s of the dimensions that is associated with
    // the normalized coordinates sorted in non-increasing order.
    for (uint d = 0; d < DIM; d++)
        p[d] = std::pair<double, uint>(normcoord[d], d);
    std::sort(p.begin(), p.end(), [](std::pair<double, uint> a, std::pair<double, uint> b) {return a.first > b.first;});

    // Convert the "bottom left" corner to a flat index k.
    uint k = this->convertIndex(idx);

    // Interpolate the return value in the simplex from the sorted dimensions.
    double y = Y.value(k)*(1-p[0].first); // the first one is special
    double cc = C.value(k)*(1-p[0].first);
    for (uint d = 1; d < DIM; d++)
    {
        //index[s[d-1].second] += 1; // this is what's written in the paper
        //k = indexToFlatIndex(index);
        k += this->cumN[p[d-1].second]; // a faster version that maintains the flat index k
        y += Y.value(k)*(p[d-1].first-p[d].first); // fast access to flat Y array
        cc += C.value(k)*(p[d-1].first-p[d].first);
    }
    //index[s[DIM-1].second] += 1;
    //k = indexToFlatIndex(index);
    k += this->cumN[p[DIM-1].second]; // maintain the flat index
    y += Y.value(k)*p[DIM-1].first; // the last one is also special
    cc += C.value(k)*p[DIM-1].first;

    c = cc;

    return y;
}

// Evaluates the SparseGridNet at point x using the output value of the nearest
// grid node. x is truncated to lie inside the boundaries of the SparseGridNet.
// If no data is loaded, this method returns 0.
double SparseGridNet::evaluateAtConst(const double* x) const
{
    // Calculate the nearest node that contains x.
    uint n = this->getNodeFlatIndex(x);

    // Return the node output.
    return Y.value(n);
}

// Evaluates the SparseGridNet at point x using the output value of the nearest
// grid node. This version also sets a confidence estimate c in [0,1].
// x is truncated to lie inside the boundaries of the SparseGridNet.
// If no data is loaded, this method returns 0.
double SparseGridNet::evaluateAtConst(const double* x, double& c) const
{
    // Calculate the "bottom left" corner of the cell that contains x.
    uint n = this->getNodeFlatIndex(x);

    // Return the node output.
    c = C.value(n);
    return Y.value(n);
}

// QPainter drawing code that draws a 2D slice of the SparseGridNet using the ColorTool height map.
void SparseGridNet::drawOutputHeightMap(QPainter *painter, double min, double max, double opacity)
{
    if (Y.isEmpty())
        return;

    QMutexLocker locker(&mutex);

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);
    for (uint j = 0; j < this->N[1]; j++)
    {
        for (uint i = 0; i < this->N[0]; i++)
        {
            uint n = i + j*this->cumN[1];
            if (C.value(n) > 0)
            {
                //painter->setPen(colorUtil.getHeightMapColor(Y[n], min, max));
                painter->setBrush(colorUtil.getHeightMapColor(Y[n], min, max));
                painter->drawRect(QRectF(this->raster[0][i]-0.5*this->stride[0], this->raster[1][j]-0.5*this->stride[1], this->stride[0], this->stride[1]));
            }
        }
    }

    painter->restore();
}

// QPainter drawing code that draws a 2D slice of the SparseGridNet using the ColorTool heat map.
void SparseGridNet::drawOutputHeatMap(QPainter *painter, double min, double max, double opacity)
{
    if (Y.isEmpty())
        return;

    QMutexLocker locker(&mutex);

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);
    for (uint j = 0; j < this->N[1]; j++)
    {
        for (uint i = 0; i < this->N[0]; i++)
        {
            uint n = i + j*this->cumN[1];
            if (C.value(n) > 0)
            {
                //painter->setPen(colorUtil.getHeatMapColor(Y[n], min, max));
                painter->setBrush(colorUtil.getHeatMapColor(Y[n], min, max));
                painter->drawRect(QRectF(this->raster[0][i]-0.5*this->stride[0], this->raster[1][j]-0.5*this->stride[1], this->stride[0], this->stride[1]));
            }
        }
    }

    painter->restore();
}
