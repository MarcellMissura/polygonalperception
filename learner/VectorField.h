#ifndef VECTORFIELD_H_
#define VECTORFIELD_H_
#include "Grid.h"
#include <util/NVec.h>
#include <util/ColorUtil.h>
#include "GL/gl.h"
#include "util/GLlib.h"
#include <QFile>


// The VectorField extends the Grid with output values at the nodes.
// Linear interpolation is used to query a value anywhere within the grid boundaries.

// The general use of the VectorField would be to first define the min and max
// data range boundaries, N the number of VectorField nodes per dimension, and
// the pinch parameter in order to construct the VectorField in the data space.
// Use setMinMax(), setN(), setPinch() for this purpose. Then, populate
// the VectorField either from a text file using loadTxt(), or by feeding it
// with data using the setNodeOutputs() method. Please understand that
// after the VectorField has been populated, it is not a good idea to change
// the VectorField structure parameters.

// After the VectorField has been constructed and populated, you can query it
// at an arbitrary location x using the evaluateAt(x) function.

template <int DIMIN=3, int DIMOUT=3>
class VectorField : public Grid<DIMIN>
{

protected:

    QHash<unsigned int, NVec<DIMOUT> > Y; // Ouput vetors of all grid nodes in a hash (size: pow(N, DIM)-ish).
    QHash<unsigned int, double> C; // Confidence values of all VectorField nodes in a flat list.

public:

    VectorField();
    ~VectorField(){}

    virtual void reset(); // Resets to a blank state.

    virtual void load(QString name = "VectorField.vf");
    virtual void save(QString name = "VectorField.vf");

    // Function approximation interface.
    void setNodeOutput(const QVector<int> idx, const NVec<DIMOUT> y, double c=1.0);
    void setNodeOutput(int idx, const NVec<DIMOUT> y, double c=1.0);
    NVec<DIMOUT> getNodeOutput(const QVector<int> idx);
    NVec<DIMOUT> getNodeOutput(int idx);

    // Query interface.
    NVec<DIMOUT> evaluateAt(NVec<DIMIN> x);
    NVec<DIMOUT> evaluateAt(NVec<DIMIN> x, double& c);
    NVec<DIMOUT> evaluateAtConst(NVec<DIMIN> x);
    NVec<DIMOUT> evaluateAtConst(NVec<DIMIN> x, double& c);
};

template <int DIMIN, int DIMOUT>
VectorField<DIMIN, DIMOUT>::VectorField()
{

}

// Resets the data grid to a blank state: zero output, zero confidence.
template <int DIMIN, int DIMOUT>
void VectorField<DIMIN, DIMOUT>::reset()
{
    Y.clear();
    C.clear();
}

// Loads a binary saved VectorField.
template <int DIMIN, int DIMOUT>
void VectorField<DIMIN, DIMOUT>::load(QString name)
{
    Grid<DIMIN>::load(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "VectorField::load(): invalid file name" << name;
        return;
    }
    fileName += ".vf";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "VectorField::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> Y;
    in >> C;
    file.close();
}

// Saves the VectorField in a binary file.
template <int DIMIN, int DIMOUT>
void VectorField<DIMIN, DIMOUT>::save(QString name)
{
    Grid<DIMIN>::save(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "VectorField::save(): invalid file name" << name;
        return;
    }
    fileName += ".vf";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "VectorField::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << Y;
    out << C;
    file.close();
}

// Sets the output value of the VectorField node identified by flat index n to y,
// and the confidence of the output to c in [0,1], 1 most confident.
template <int DIMIN, int DIMOUT>
void VectorField<DIMIN, DIMOUT>::setNodeOutput(int idx, NVec<DIMOUT> y, double c)
{
    Y[idx] = y;
    C[idx] = c;
}

// Sets the output value of the VectorField node identified by DIM index idx to y,
// and the confidence of the output to c.
template <int DIMIN, int DIMOUT>
void VectorField<DIMIN, DIMOUT>::setNodeOutput(const QVector<int> idx, NVec<DIMOUT> y, double c)
{
    int n = this->convertIndex(idx);
    Y[n] = y;
    C[n] = c;
}

// Returns the VectorField node output value of the node identified by flat index idx.
template <int DIMIN, int DIMOUT>
NVec<DIMOUT> VectorField<DIMIN, DIMOUT>::getNodeOutput(int idx)
{
    return Y[idx];
}

// Returns the VectorField node output value of the node identified by DIM index idx.
template <int DIMIN, int DIMOUT>
NVec<DIMOUT> VectorField<DIMIN, DIMOUT>::getNodeOutput(const QVector<int> idx)
{
    int n = this->convertIndex(idx);
    return Y[n];
}

// Evaluates the VectorField at point x using linear interpolation between the
// grid nodes of the simplex that envelopes x. x is truncated to lie inside
// the boundaries of the VectorField. The computations in this method are carried
// out according to the paper "A Geometric Approach to Maximum-Speed
// n-Dimensional Continuous Linear Interpolation in Rectangular VectorFields",
// Riccardo Rovatti, Michele Borgatti, and Roberto Guerrier. If no data is
// loaded, this method returns 0.
template <int DIMIN, int DIMOUT>
NVec<DIMOUT> VectorField<DIMIN, DIMOUT>::evaluateAt(NVec<DIMIN> x)
{
    if (this->nodeCount != Y.size())
        return 0;

    // Bound X.
    x.bound(this->min, this->max);

    // Calculate the "bottom left" corner of the cell that contains X.
    QVector<int> index = Grid<DIMIN>::getNodeIndexBl(x); // make sure to use an untransformed version

    // Calculate the normalized coordinates of X within the cell.
    double normcoord[DIMIN];
    for (int d = 0; d < DIMIN; d++)
        normcoord[d] = (x[d]-this->raster[d][index[d]])/(this->raster[d][index[d]+1]-this->raster[d][index[d]]);

    // Compute the permutation s of the dimensions that is associated with
    // the normalized coordinates sorted in non-increasing order.
    std::array<std::pair<double, int>, DIMIN> p;
    for (int d = 0; d < DIMIN; d++)
        p[d] = std::pair<double, int>(normcoord[d], d);
    std::sort(p.begin(), p.end(), [](std::pair<double, int> a, std::pair<double, int> b) {return a.first > b.first;});

    // Convert the "bottom left" corner to a flat index k.
    int k = this->convertIndex(index);

    // Interpolate the return value in the simplex from the sorted dimensions.
    NVec<DIMOUT> y = Y[k]*(1-p[0].first); // the first one is special

    for (int d = 1; d < DIMIN; d++)
    {
        //index[s[d-1].second] += 1; // this is what's written in the paper
        //k = indexToFlatIndex(index);
        k += this->cumN[p[d-1].second]; // a faster version that maintains the flat index k
        y += Y[k]*(p[d-1].first-p[d].first); // fast access to flat Y array
    }

    //index[s[DIM-1].second] += 1;
    //k = indexToFlatIndex(index);
    k += this->cumN[p[DIMIN-1].second]; // maintain the flat index
    y += Y[k]*p[DIMIN-1].first; // the last one is also special

    return y;
}

// Evaluates the VectorField at point x using linear interpolation between the
// grid nodes of the simplex that envelopes x. x is truncated to lie inside
// the boundaries of the VectorField. The computations in this method are carried
// out according to the paper "A Geometric Approach to Maximum-Speed
// n-Dimensional Continuous Linear Interpolation in Rectangular VectorFields",
// Riccardo Rovatti, Michele Borgatti, and Roberto Guerrier. If no data is
// loaded, this method returns 0.
template <int DIMIN, int DIMOUT>
NVec<DIMOUT> VectorField<DIMIN, DIMOUT>::evaluateAt(NVec<DIMIN> x, double& c)
{
    if (this->nodeCount != Y.size())
        return 0;

    // Bound X.
    x.bound(this->min, this->max);

    // Calculate the "bottom left" corner of the cell that contains X.
    QVector<int> index = Grid<DIMIN>::getNodeIndexBl(x); // make sure to use an untransformed version

    // Calculate the normalized coordinates of X within the cell.
    double normcoord[DIMIN];
    for (int d = 0; d < DIMIN; d++)
        normcoord[d] = (x[d]-this->raster[d][index[d]])/(this->raster[d][index[d]+1]-this->raster[d][index[d]]);

    // Compute the permutation s of the dimensions that is associated with
    // the normalized coordinates sorted in non-increasing order.
    std::array<std::pair<double, int>, DIMIN> p;
    for (int d = 0; d < DIMIN; d++)
        p[d] = std::pair<double, int>(normcoord[d], d);
    std::sort(p.begin(), p.end(), [](std::pair<double, int> a, std::pair<double, int> b) {return a.first > b.first;});

    // Convert the "bottom left" corner to a flat index k.
    int k = this->convertIndex(index);

    // Interpolate the return value in the simplex from the sorted dimensions.
    NVec<DIMOUT> y = Y[k]*(1-p[0].first); // the first one is special
    double cc = C[k]*(1-p[0].first);
    for (int d = 1; d < DIMIN; d++)
    {
        //index[s[d-1].second] += 1; // this is what's written in the paper
        //k = indexToFlatIndex(index);
        k += this->cumN[p[d-1].second]; // a faster version that maintains the flat index k
        y += Y[k]*(p[d-1].first-p[d].first); // fast access to flat Y array
        cc += C[k]*(p[d-1].first-p[d].first);
    }

    //index[s[DIM-1].second] += 1;
    //k = indexToFlatIndex(index);
    k += this->cumN[p[DIMIN-1].second]; // maintain the flat index
    y += Y[k]*p[DIMIN-1].first; // the last one is also special
    cc += C[k]*p[DIMIN-1].first;

    c = cc;

    return y;
}


// Evaluates the VectorField at point x using the output value of the nearest
// grid node. x is truncated to lie inside the boundaries of the VectorField.
// If no data is loaded, this method returns 0.
template <int DIMIN, int DIMOUT>
NVec<DIMOUT> VectorField<DIMIN, DIMOUT>::evaluateAtConst(NVec<DIMIN> x)
{
    if (this->nodeCount != Y.size())
        return 0;

    // Bound x.
    x.bound(this->min, this->max);

    // Calculate the "bottom left" corner of the cell that contains x.
    QVector<int> idx = Grid<DIMIN>::getNodeIndex(x); // make sure to use an untransformed version

    // Convert the cell index to a flat index.
    int n = this->convertIndex(idx);

    // And return the node output.
    return Y[n];
}

// Evaluates the VectorField at point x using the output value of the nearest
// grid node. This version also sets a confidence estiamte c in [0,1].
// x is truncated to lie inside the boundaries of the VectorField.
// If no data is loaded, this method returns 0.
template <int DIMIN, int DIMOUT>
NVec<DIMOUT> VectorField<DIMIN, DIMOUT>::evaluateAtConst(NVec<DIMIN> x, double& c)
{
    if (this->nodeCount != Y.size())
        return 0;

    // Bound x.
    x.bound(this->min, this->max);

    // Calculate the "bottom left" corner of the cell that contains x.
    QVector<int> idx = Grid<DIMIN>::getNodeIndex(x); // make sure to use an untransformed version

    // Convert the cell index to a flat index.
    int n = this->convertIndex(idx);

    // And return the node output.
    c = C[n];
    return Y[n];
}

#endif /* VectorField_H_ */
