#ifndef ADJACENCYMATRIX_H_
#define ADJACENCYMATRIX_H_
#include "util/Vector.h"
class AdjacencyMatrix
{
    Vector< Vector<char> > m;

public:

    AdjacencyMatrix();
    ~AdjacencyMatrix(){}

    void clear();
    void set(uint i, uint j);
    void unset(uint i, uint j);
    bool isSet(uint i, uint j) const;
};

#endif
