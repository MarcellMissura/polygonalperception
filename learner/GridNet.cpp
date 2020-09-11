#include "GridNet.h"
#include <GL/glu.h>
#include "util/GLlib.h"
#include "util/ColorUtil.h"
#include "blackboard//Config.h"
#include "globals.h"
#include <QFile>
#include <QMutexLocker>
#include <QDebug>
#include "blackboard/State.h"


// The GridNet extends the Grid with output values at the grid nodes.
// Linear interpolation is used to query a value anywhere within the grid boundaries.
// The GridNet allocates memory for all grid nodes. This results in fast lookups, but
// it has a large memory footprint. If you run into memory issues, try the SparseGridNet.

// The general use of the GridNet would be to first define the min and max
// data range boundaries and N the number of GridNet nodes per dimension.
// Use setMinMax() and setN() for this purpose. In fact, you must Then, populate
// the GridNet from saved data using load(), or by feeding it
// with data using the setNodeOutputs() method. Please understand that
// after the GridNet has been populated, it is not a good idea to change
// the GridNet structure parameters.

// After the GridNet has been constructed and populated, you can query it
// at an arbitrary location x using the evaluateAt(x) function.

GridNet::GridNet()
{

}

// Calculates the raster of the grid coordinates. The grid nodes are distributed between the
// respective min and max values of each dimension such that the first node is located at the
// min and the last node is located at the max. Dim, N, min, and max must be set before
// computing the raster. Make sure to set the grid parameters first and then call this method
// to prepare the grid before using it.
void GridNet::rasterize()
{
    // GridNet overrides Grid::rasterize() so that we can resize Y and C and p here.
    Grid::rasterize();
    Y.resize(this->getNodeCount());
    C.resize(this->getNodeCount());
    p.resize(this->getNodeCount());
}

// Resets the data grid to a blank state: zero output, zero confidence,
// but does not change the layout of the data grid.
void GridNet::reset()
{
    //Y.fill(0);
    //C.fill(0);
    Y.assign(Y.size(), 0);
    C.assign(C.size(), 0);
}

// Loads a binary saved GridNet.
void GridNet::load(QString name)
{
    Grid::load(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "GridNet::load(): invalid file name" << name;
        return;
    }
    fileName += ".gnt";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "GridNet::load(): Could not open file" << file.fileName();
        return;
    }

//    QDataStream in(&file);
//    in >> Y;
//    in >> C;
//    file.close();
}

// Saves the GridNet in a binary file.
void GridNet::save(QString name)
{
    Grid::save(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "GridNet::save(): invalid file name" << name;
        return;
    }
    fileName += ".gnt";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "GridNet::save(): Could not write to file" << file.fileName();
        return;
    }

//    QDataStream out(&file);
//    out << Y;
//    out << C;
//    file.close();
}

// Sets the output value of the GridNet node identified by flat index n to y,
// and the confidence of the output to c in [0,1], 1 most confident.
void GridNet::setNodeOutput(uint n, double y, double c)
{
    Y[n] = y;
    C[n] = c;
}

// Sets the output value of the GridNet node identified by DIM index idx to y,
// and the confidence of the output to c.
void GridNet::setNodeOutput(const uint *idx, double y, double c)
{
    uint n = this->convertIndex(idx);
    Y[n] = y;
    C[n] = c;
}

// Returns the GridNet node output value of the node identified by flat index n.
double GridNet::getNodeOutput(uint n) const
{
    return Y[n];
}

// Returns the GridNet node output value of the node identified by DIM index idx.
double GridNet::getNodeOutput(const uint *idx) const
{
    uint n = this->convertIndex(idx);
    return Y[n];
}

// Returns the GridNet node confidence value of the node identified by flat index idx.
double GridNet::getNodeConfidence(uint idx) const
{
    return C[idx];
}

// Returns the GridNet node confidence value of the node identified by DIM index idx.
double GridNet::getNodeConfidence(const uint *idx) const
{
    uint n = this->convertIndex(idx);
    return C[n];
}

// Text console output of the full GridNet including the output.
void GridNet::printGridNet(uint howMany) const
{
    uint top = howMany == 0 ? Y.size() : qMax(howMany, (uint)Y.size());
    for (uint i = 0; i < top; i++)
        qDebug() << i << this->getNodeCoordinates(i) << "| y:" << Y[i] << "c:" << C[i];
}

// Evaluates the GridNet at point x using linear interpolation between the
// grid nodes of the simplex that envelopes x. x is truncated to lie inside
// the boundaries of the GridNet. The computations in this method are carried
// out according to the paper "A Geometric Approach to Maximum-Speed
// n-Dimensional Continuous Linear Interpolation in Rectangular GridNets",
// Riccardo Rovatti, Michele Borgatti, and Roberto Guerrier.
double GridNet::evaluateAt(const double *x) const
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
    double y = Y.at(k)*(1-p[0].first); // the first one is special
    for (uint d = 1; d < DIM; d++)
    {
        //index[s[d-1].second] += 1; // this is what's written in the paper
        //k = indexToFlatIndex(index);
        k += this->cumN[p[d-1].second]; // a faster version that maintains the flat index k
        y += Y.at(k)*(p[d-1].first-p[d].first); // fast access to flat Y array
    }
    //index[s[DIM-1].second] += 1;
    //k = indexToFlatIndex(index);
    k += this->cumN[p[DIM-1].second]; // maintain the flat index
    y += Y.at(k)*p[DIM-1].first; // the last one is also special

    return y;
}

// Evaluates the GridNet at point x using linear interpolation between the
// grid nodes of the simplex that envelopes x. x is truncated to lie inside
// the boundaries of the GridNet. The computations in this method are carried
// out according to the paper "A Geometric Approach to Maximum-Speed
// n-Dimensional Continuous Linear Interpolation in Rectangular GridNets",
// Riccardo Rovatti, Michele Borgatti, and Roberto Guerrier. If no data is
// loaded, this method returns 0.
double GridNet::evaluateAt(const double *x, double& c) const
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
    double y = Y.at(k)*(1-p[0].first); // the first one is special
    double cc = C.at(k)*(1-p[0].first);
    for (uint d = 1; d < DIM; d++)
    {
        //index[s[d-1].second] += 1; // this is what's written in the paper
        //k = indexToFlatIndex(index);
        k += this->cumN[p[d-1].second]; // a faster version that maintains the flat index k
        y += Y.at(k)*(p[d-1].first-p[d].first); // fast access to flat Y array
        cc += C.at(k)*(p[d-1].first-p[d].first);
    }
    //index[s[DIM-1].second] += 1;
    //k = indexToFlatIndex(index);
    k += this->cumN[p[DIM-1].second]; // maintain the flat index
    y += Y.at(k)*p[DIM-1].first; // the last one is also special
    cc += C.at(k)*p[DIM-1].first;

    c = cc;

    return y;
}

// Evaluates the GridNet at point x using the output value of the nearest
// grid node. x is truncated to lie inside the boundaries of the GridNet.
// If no data is loaded, this method returns 0.
double GridNet::evaluateAtConst(const double *x) const
{
    // Calculate the nearest node that contains x.
    uint n = this->getNodeFlatIndex(x);

    // Return the node output.
    return Y[n];
}

// Evaluates the GridNet at point x using the output value of the nearest
// grid node. This version also sets a confidence estiamte c in [0,1].
// x is truncated to lie inside the boundaries of the GridNet.
// If no data is loaded, this method returns 0.
double GridNet::evaluateAtConst(const double *x, double& c) const
{
    // Calculate the "bottom left" corner of the cell that contains x.
    uint n = this->getNodeFlatIndex(x);

    // Return the node output.
    c = C[n];
    return Y[n];
}

// QPainter drawing code that draws a 2D slice of the GridNet using the ColorTool height map.
void GridNet::drawOutputHeightMap(QPainter *painter, double min, double max, double opacity)
{
    if (Y.empty())
        return;

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);
    for (uint j = 0; j < this->N[1]; j++)
    {
        for (uint i = 0; i < this->N[0]; i++)
        {
            uint n = i + j*this->cumN[1];
            if (C[n] > 0)
            {
                //painter->setPen(colorUtil.getHeightMapColor(Y[n], min, max));
                painter->setBrush(colorUtil.getHeightMapColor(Y[n], min, max));
                painter->drawRect(QRectF(this->raster[0][i]-0.5*this->stride[0], this->raster[1][j]-0.5*this->stride[1], this->stride[0], this->stride[1]));
            }
        }
    }

    painter->restore();
}

void GridNet::drawOutputHeightMap3d(double min, double max, double opacity)
{
    if (Y.empty())
        return;

    glPushMatrix();
    glBegin(GL_QUADS);

    for (uint j = 0; j < this->N[1]; j++)
    {
        for (uint i = 0; i < this->N[0]; i++)
        {
            uint n = i + j*this->cumN[1];
            if (C[n] > 0)
            {
//                painter->setBrush(colorUtil.getHeightMapColor(Y[n], min, max));
                QColor c = colorUtil.getHeightMapColor(Y[n], min, max);
                glColor4f(c.redF(), c.greenF(), c.blueF(), opacity);

//                painter->drawRect(QRectF(this->raster[0][i]-0.5*this->stride[0], this->raster[1][j]-0.5*this->stride[1], this->stride[0], this->stride[1]));

                double x, y, w, h;
                x = this->raster[0][i]-0.5*this->stride[0];
                y = this->raster[1][j]-0.5*this->stride[1];
                w = this->stride[0];
                h = this->stride[1];

                glVertex3f(x,y,0);
                glVertex3f(x+w,y,0);
                glVertex3f(x+w,y+h,0);
                glVertex3f(x,y+h,0);
            }
        }
    }

    glEnd();
    glPopMatrix();
}

// QPainter drawing code that draws a 2D slice of the GridNet using the ColorUtil heat map.
void GridNet::drawOutputHeatMap(QPainter *painter, double min, double max, double opacity)
{
    if (Y.empty())
        return;

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);
    for (uint j = 0; j < this->N[1]; j++)
    {
        for (uint i = 0; i < this->N[0]; i++)
        {
            uint n = i + j*this->cumN[1];
            if (C[n] > 0)
            {
                //painter->setPen(colorUtil.getHeatMapColor(Y[n], min, max));
                painter->setBrush(colorUtil.getHeatMapColor(Y[n], min, max));
                painter->drawRect(QRectF(this->raster[0][i]-0.5*this->stride[0], this->raster[1][j]-0.5*this->stride[1], this->stride[0], this->stride[1]));
            }
        }
    }

    painter->restore();
}
