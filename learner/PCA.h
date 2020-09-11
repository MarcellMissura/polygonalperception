#ifndef PCA_H_
#define PCA_H_
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>
#include "util/Vector.h"
#include "util/Vec3.h"
#include "util/Vec2.h"

// This is an ordinary (linear) least squares regressor.
// https://en.wikipedia.org/wiki/Ordinary_least_squares
// Use addDataPoint() to feed the OLS with data. Then, use init() to initialze
// the regression parameters. After initialization, you can use evaluateAt(x) to
// query for an estimate at location x, or getNormal() to ask for the plane normal.

class PCA
{
    int loadedPoints; // number of actually loaded points
    Vector<Vec3> data;
    arma::mat coeff;
    Vec3 normal;
    Vec3 mean;

public:

    PCA();
    ~PCA(){}

    void init();
    void reset();

    int getLoadedPointCount() const;
    void addDataPoint(const Vec3& p);
    double evaluateAt(const Vec2 &p) const;
    Vec3 getNormal() const;
    Vec3 getMean() const;

    // OpenGL drawing code.
    void draw(uint sampleFactor=1) const;

    void print() const;

};

QDebug operator<<(QDebug dbg, const PCA &o);

#endif // OLS
