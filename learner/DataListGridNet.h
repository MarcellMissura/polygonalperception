#ifndef DATALISTGRIDNET_H_
#define DATALISTGRIDNET_H_
#include "GridNet.h"
#include <util/VecN.h>
#include <util/VecNu.h>
#include <util/Vector.h>
#include <util/ColorUtil.h>
#include "util/GLlib.h"
#include "util/Statistics.h"
//#include "u
#include <QFile>
#include <QMutex>

// The DataListGridNet extends the GridNet with data storage capabilities.
// Data points can be added to the grid. They will be sorted into bins
// assigned to the grid nodes. Each bin can contain a list of data points

class DataListGridNet : public GridNet
{

protected:

    Vector< Vector<uint> > bin; // Data bins assigned to the grid nodes.

public:

    DataListGridNet();
    ~DataListGridNet(){}

    void reset(); // Resets to a blank state.
    void rasterize();

    void addDataPoint(uint n, uint y);
    const Vector<uint>* getDataPoints(uint n) const;
};

#endif /* DATALISTGRIDNET_H_ */

