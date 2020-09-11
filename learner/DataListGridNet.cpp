#include "DataListGridNet.h"

DataListGridNet::DataListGridNet()
{

}

// Resets the data grid to a blank state: zero output, zero confidence.
void DataListGridNet::reset()
{
    GridNet::reset();
    for (int i = 0; i < bin.size(); i++)
        bin[i].clear();
}

void DataListGridNet::rasterize()
{
    GridNet::rasterize();
    bin.resize(getNodeCount());
}

// Adds a single data point with input values x and output value y to the data set.
// The method returns the flat index of the grid node the data point was assigned to.
// Out of bounds points are ignored and -1 is returned.
void DataListGridNet::addDataPoint(uint n, uint y)
{
    // Discard NaN cases.
    if (y != y)
        return;

    // Discard out of range points.
    if (n < 0 || n >= nodeCount)
        return;

    // Append to the data bin.
    bin[n].push_back(y);
}

// Returns the data points of the grid node identified by the flat index n.
const Vector<uint>* DataListGridNet::getDataPoints(uint n) const
{
    return &(bin[n]);
}
