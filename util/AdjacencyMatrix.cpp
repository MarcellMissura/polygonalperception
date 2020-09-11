#include "AdjacencyMatrix.h"

// This is a memory preserving implementation of an adjacency matrix
// where any entry (i,j) of the matrix can be 0 or 1. Memory preserving
// means that the matrix grows automatically in size when needed, but
// memory is never released again to avoid reoccuring heap allocations.

AdjacencyMatrix::AdjacencyMatrix()
{

}

// Resets the matrix to all zeros.
void AdjacencyMatrix::clear()
{
    for (int i = 0; i < m.size(); i++)
        m[i].fill(0);
}

// Set the matrix entry at (i,j) to 1.
// The matrix will automatically grow if it is too small
// to accomodate an element i,j.
void AdjacencyMatrix::set(uint i, uint j)
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    m.ensureSize(i+1);
    m[i].ensureSize(j+1);
    m[i][j] = 1;
}

// Set the matrix entry at (i,j) to 0.
// This operation will not cause the matrix to grow.
void AdjacencyMatrix::unset(uint i, uint j)
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i < m.size() && j < m[i].size())
        m[i][j] = 0;
}

// Check if the matrix entry at (i,j) is set to 1.
// This operation will not cause the matrix to grow
// and returns false if i,j is out of bounds.
bool AdjacencyMatrix::isSet(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i+1 > m.size() || j+1 > m[i].size())
        return false;
    return (m[i][j] > 0);
}

