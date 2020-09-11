#ifndef SPARSEGRIDNET_H_
#define SPARSEGRIDNET_H_
#include "Grid.h"
#include <QMutex>


// The SparseGridNet extends the Grid with output values at the grid nodes. Linear
// interpolation is used to query a value anywhere within the grid boundaries.
// As opposed to the GridNet that allocates all needed memory at grid contruction
// time, the SparseGridNet uses a QHash for storage and minimizes the memory
// footprint at the expense of slower lookups.

// To use the SparseGridNet, first set up the grid by providing the grid parameters and calling
// rasterize(). Example:
//
// SparseGridNet grid;
// grid.setDim(3); // Set the number of dimensions.
// uint N[3] = {101,201,301};
// grid.setN(N); // Set the numer of nodes per dimension.
// double min[3] = {config.xMin, config.yMin, config.zMin};
// grid.setMin(min); // Set the minimum values per dimension.
// double max[3] = {config.xMax, config.yMax, config.zMax};
// grid.setMax(max); // Set the maximum values per dimension.
// grid.rasterize(); // Compute the grid representation.
//
// Then, populate the grid by setting the node outputs.
//
// for (int n = 0; n < grid.getNodeCount(); n++)
//      grid.setNodeOutput(n, some value);
//
// Then, you can interpolate any point within the grid bounds.
//
// double y = grid.evaluateAt(x);
//
// You can also provide and retrieve a confidence value in [0,1] for each node, which will
// also be linearly interpolated for an arbitrary location in the grid.
//
// for (int n = 0; n < grid.getNodeCount(); n++)
//      grid.setNodeOutput(n, some value, some confidence);
//
// double c;
// double y = grid.evaluateAt(x, c);

class SparseGridNet : public Grid
{
    QMutex mutex;

protected:

    QHash<uint, double> Y; // Ouput values of all SparseGridNet nodes in a flat list (size: pow(N, DIM)-ish).
    QHash<uint, double> C; // Confidence values of all SparseGridNet nodes in a flat list.

private:
    mutable std::vector<std::pair<double, uint> > p;

public:

    SparseGridNet();
    virtual ~SparseGridNet(){}

    void rasterize();

    virtual void reset(); // Resets to a blank state.

    virtual void load(QString name = "SparseGridNet.sgnt");
    virtual void save(QString name = "SparseGridNet.sgnt");

    // Function approximation interface.
    void setNodeOutput(const uint* idx, double y, double c=1.0);
    void setNodeOutput(uint n, double y, double c=1.0);
    double getNodeOutput(const uint* idx) const;
    double getNodeOutput(uint n) const;
    double getNodeConfidence(const uint* idx) const;
    double getNodeConfidence(uint n) const;
    void printSparseGridNet(uint howMany=0) const;  // Text console output of the full GridNet including the output.

    // Query interface.
    double evaluateAt(const double* x) const;
    double evaluateAt(const double* x, double& c) const;
    double evaluateAtConst(const double* x) const;
    double evaluateAtConst(const double* x, double& c) const;

    // QPainter drawing code.
    void drawOutputHeightMap(QPainter *painter, double min=0, double max=1.0, double opacity=1.0);
    void drawOutputHeatMap(QPainter *painter, double min=0, double max=1.0, double opacity=1.0);
};

#endif /* SPARSEGRIDNET_H_ */
