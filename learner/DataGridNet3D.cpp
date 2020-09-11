#include "DataGridNet3D.h"
#include <QGLViewer/vec.h>
using namespace qglviewer;

DataGridNet3D::DataGridNet3D()
{

}

void DataGridNet3D::reset()
{
    DataGridNet<3>::reset();
    this->transform.setPositionAndOrientation(Vec(), Quaternion());
}

// Sets the transformation of the grid with respect to the world coordinate frame.
void DataGridNet3D::setTransform(double r, double p, double y)
{
    // Please note that the grid transformation has no effect on the computation of
    // the grid coordinates. The grid is stored in an "untransformed" state, i.e.
    // the raster remains the same. evaluateAt() will map the queried data point
    // using the inverse transform before looking it up in the grid, and
    // getNodeCoordinates() transforms the node coordinates before returning it.

    Frame frame;
    frame.rotate(Quaternion(Vec(1,0,0), r));
    frame.rotate(Quaternion(Vec(0,1,0), p));
    frame.rotate(Quaternion(Vec(0,0,1), y));
    this->transform = frame;
}

// Sets the transformation of the grid with respect to the world coordinate frame.
void DataGridNet3D::setTransform(qglviewer::Frame tr)
{
    // Please note that the grid transformation has no effect on the computation of
    // the grid coordinates. The grid is stored in an "untransformed" state, i.e.
    // the raster remains the same. evaluateAt() will map the queried data point
    // using the inverse transform before looking it up in the grid, and
    // getNodeCoordinates() transforms the node coordinates before returning it.

    this->transform = tr;
}

// Returns the grid transform.
qglviewer::Frame DataGridNet3D::getTransform()
{
    return transform;
}

// Loads a binary saved grid.
void DataGridNet3D::load(QString name)
{
    DataGridNet<3>::load(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "DataGridNet3D::load(): invalid file name" << name;
        return;
    }
    fileName += ".tra";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "DataGridNet3D::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    double x,y,z,q0,q1,q2,q3;
    in >> x;
    in >> y;
    in >> z;
    in >> q0;
    in >> q1;
    in >> q2;
    in >> q3;
    transform.setPosition(x,y,z);
    transform.setOrientation(q0,q1,q2,q3);
    file.close();
}

// Saves the grid in a binary file.
void DataGridNet3D::save(QString name)
{
    DataGridNet<3>::save(name);

    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "DataGridNet3D::save(): invalid file name" << name;
        return;
    }
    fileName += ".tra";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "DataGridNet3D::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << transform.position().x;
    out << transform.position().y;
    out << transform.position().z;
    out << transform.orientation()[0];
    out << transform.orientation()[1];
    out << transform.orientation()[2];
    out << transform.orientation()[3];
    file.close();
}


// Returns the world coordinates of the grid node specified by the DIM dimensional index.
NVec<3> DataGridNet3D::getNodeCoordinates(const QVector<uint> idx)
{
    NVec<3> v = Grid<3>::getNodeCoordinates(idx);
    Vec tr = transform.inverseCoordinatesOf(Vec(v.x, v.y, v.z));
    return NVec<3>(tr.x, tr.y, tr.z);
}

// Returns the world coordinates of the grid node specified by the flat index.
NVec<3> DataGridNet3D::getNodeCoordinates(uint idx)
{
    NVec<3> v = Grid<3>::getNodeCoordinates(idx);
    Vec tr = transform.inverseCoordinatesOf(Vec(v.x, v.y, v.z));
    return NVec<3>(tr.x, tr.y, tr.z);
}

// Computes the dim index of the grid node closest to the point x.
QVector<uint> DataGridNet3D::getNodeIndex(const NVec<3> &x)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::getNodeIndex(NVec<3>(trx.x, trx.y, trx.z));
}

// Computes the "bottom left" dim index of the grid node that contains point x.
QVector<uint> DataGridNet3D::getNodeIndexBl(const NVec<3> &x)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::getNodeIndexBl(NVec<3>(trx.x, trx.y, trx.z));
}

// Computes the flat index of the grid node closest to the point x.
uint DataGridNet3D::getNodeFlatIndex(const NVec<3> &x)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::getNodeFlatIndex(NVec<3>(trx.x, trx.y, trx.z));
}

// Computes the "bottom left" flat index of the grid node that contains point x.
uint DataGridNet3D::getNodeFlatIndexBl(const NVec<3> &x)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::getNodeFlatIndexBl(NVec<3>(trx.x, trx.y, trx.z));
}

// Returns true if the given cartesian point is within the boundaries of the grid.
bool DataGridNet3D::containsPoint(const NVec<3> x)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::containsPoint(NVec<3>(trx.x, trx.y, trx.z));
}

// Returns a uniformly sampled point from the grid space.
NVec<3> DataGridNet3D::samplePoint()
{
    NVec<3> v = Grid<3>::samplePoint();
    Vec tr = transform.inverseCoordinatesOf(Vec(v.x, v.y, v.z));
    return NVec<3>(tr.x, tr.y, tr.z);
}

// Adds a single data point with input values x and output value y to the data set.
// The method returns the flat index of the grid node the data point was assigned to.
uint DataGridNet3D::addDataPoint(NVec<3> x, double y)
{
    // Inverse transform the data point into the reference frame of the grid.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::addDataPoint(NVec<3>(trx.x, trx.y, trx.z), y);
}

// Returns the data points in a neighbourhood of radius r around the grid node identified by the DIM index idx.
QList< NVec<4> > DataGridNet3D::getDataPoints(const QVector<uint> idx, uint r)
{
    // Select the data points from the data grid.
    QList< NVec<4> > data = DataGridNet<3>::getDataPoints(idx, r);

    // Invert the transformation on every data point to convert them back to world coordinates.
    for (int i=0; i < data.size(); i++)
    {
        Vec tr = transform.inverseCoordinatesOf(Vec(data[i].x, data[i].y, data[i].z));
        data[i] = NVec<4>(tr.x, tr.y, tr.z, data[i].last());
    }

    return data;
}

// Returns the data points in a neighbourhood of radius r around the grid node identified by the flat index n.
QList< NVec<4> > DataGridNet3D::getDataPoints(uint idx, uint r)
{
    // Select the data points from the data grid.
    QList< NVec<4> > data = DataGridNet<3>::getDataPoints(idx, r);

    // Invert the transformation on every data point to convert them back to world coordinates.
    for (int i=0; i < data.size(); i++)
    {
        Vec tr = transform.inverseCoordinatesOf(Vec(data[i].x, data[i].y, data[i].z));
        data[i] = NVec<4>(tr.x, tr.y, tr.z, data[i].last());
    }

    return data;
}

// Returns all data points stored in the grid.
QList< NVec<4> > DataGridNet3D::getAllDataPoints()
{
    // Select the data points from the data grid.
    QList< NVec<4> > data = DataGridNet<3>::getAllDataPoints();

    // Invert the transformation on every data point to convert them back to world coordinates.
    for (int i=0; i < data.size(); i++)
    {
        Vec tr = transform.inverseCoordinatesOf(Vec(data[i].x, data[i].y, data[i].z));
        data[i] = NVec<4>(tr.x, tr.y, tr.z, data[i].last());
    }

    return data;
}

// Evaluates the grid at point X using ultra fast linear interpolation.
// X is truncated to lie inside the boundaries of the grid.
// The computations in this method are carried out according to the paper
// "A Geometric Approach to Maximum-Speed n-Dimensional Continuous Linear
// Interpolation in Rectangular Grids", Riccardo Rovatti, Michele Borgatti,
// and Roberto Guerrier. If no data are loaded, this method returns 0.
double DataGridNet3D::evaluateAt(NVec<3> x)
{
    // Inverse transform the data point into the reference frame of the grid.
    // The query point is provided in world coordinates and it must be transformed
    // into the grid coordinate frame in order to compute the cell index.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::evaluateAt(NVec<3>(trx.x, trx.y, trx.z));
}

// Evaluates the Grid at point X using ultra fast linear interpolation.
// This version also returns a confidence estiamte c in [0,1] and thus
// takes twice as long to compute.
// X is truncated to lie inside the boundaries of the grid.
// The computations in this method are carried out according to the paper
// "A Geometric Approach to Maximum-Speed n-Dimensional Continuous Linear
// Interpolation in Rectangular Grids", Riccardo Rovatti, Michele Borgatti,
// and Roberto Guerrier. If no data are loaded, this method returns 0.
double DataGridNet3D::evaluateAt(NVec<3> x, double& c)
{
    // Inverse transform the data point into the reference frame of the grid.
    // The query point is provided in world coordinates and it must be transformed
    // into the grid coordinate frame in order to compute the cell index.
    Vec trx = transform.coordinatesOf(Vec(x.x, x.y, x.z));
    return DataGridNet<3>::evaluateAt(NVec<3>(trx.x, trx.y, trx.z), c);
}

