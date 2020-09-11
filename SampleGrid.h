#ifndef SAMPLEGRID_H_
#define SAMPLEGRID_H_
#include "util/Vector.h"
#include "util/Vec2u.h"
#include "util/Vec3.h"
#include "learner/OLS.h"
#include <QPainter>

// A sample s = (p,n) is a point p and a normal n that together
// desribe a plane. A sample is gained from the RGB-D image, so
// it also has pixel coordinates, a bufferIdx in the point buffer,
// and a grid index in the sample grid.
struct Sample
{
    Vec2u imagePx;
    Vec2u gridIdx;
    int bufferIdx = 0;
    Vec3 p;
    Vec3 n;
    double angle = 0;
    bool in = true;
    int clusterId = -1;
    static Vec3 up;

    Sample()
    {
        //n.z = 1.0;
    }

    bool operator>(const Sample &o) const
    {
        return (up*p > up*o.p);
    }
    bool operator<(const Sample &o) const
    {
        return (up*p < up*o.p);
    }
    bool operator==(const Sample& o) const
    {
        return (bufferIdx == o.bufferIdx);
    }
    void operator+=(const Sample& o)
    {
        n += o.n;
        p += o.p;
    }
    void operator/=(double o)
    {
        n /= o;
        n.normalize();
        p /= o;
    }

    // The plane distance between two samples.
    // The normals have to be normalized!
    double distance(const Sample& o) const
    {
        double d1 = fabs(n*(o.p-p));
        double d2 = fabs(o.n*(o.p-p));
        double d3 = 1.0-n*o.n;
        return d1+d2;
    }

    // Distance of v to the plane described by this sample.
    double distance(const Vec3& v) const
    {
        return n*(v-p);
    }

    // Returns the plane height z at the world coordinates v.
    double evaluateAt(const Vec2& v) const
    {
        return (p*n-v.x*n.x-v.y*n.y)/n.z;
    }

    // Projects the vector v onto the plane described by this sample.
    Vec3 projectTo(const Vec3& v) const
    {
        return Vec3(v.x, v.y, evaluateAt(v));
    }
};

// The SampleGrid organizes a set of samples taken from the point cloud
// in a grid structure defined in the pixel coordinates of the camera image.
// The normals are also computed for every sample in the grid.
// Then, you can call one of the provided functions to find the floor plane.
class SampleGrid
{
    Vector< Vector<Sample> > samples; // samples in a 2D grid structure
    Vector<Sample> prunedSamples; // Only the pruned samples in a vector.

    Vector<Vector<Sample> > planes; // All plane segments.
    Vector<Sample> planeAvg; // The averages of all plane segments.
    Vector<Sample> floorSegment; // contains the set of points that make up the floor
    Sample floorPlane; // one representative of the floor plane in (normal,point) form.

    Vector<Sample> planeCluster; // temporary
    Vec3 upVector; // The up vector the samples are pruned against.
    OLS ols; // Linear fitter.

public:

    SampleGrid();
    ~SampleGrid(){}

    void init();
    void update();

    void setUpVector(const Vec3& up);
    Vec3 getUpVector() const;

    Sample findFloor();

    void drawSamples(QPainter *painter) const;
    void drawSamples() const;

private:
    void floodFill(const Vec2u &parentIdx);
    void prune();
    bool isIn(const Vec2u& gridIdx) const;

};

QDebug operator<<(QDebug dbg, const SampleGrid &o);
QDebug operator<<(QDebug dbg, const Sample &o);

#endif
